# How COPC builds levels of detail

> [index](copc_laz_lod_tutorial.md) · [1. Format](copc_format.md) · **2. LOD mechanisms** · [3. PDAL](copc_pdal.md) · [4. VTK camera](copc_vtk_camera.md) · [5. Worked examples](copc_worked_examples.md)

Extent, density, spacing and resolution; the LOD ladder; thinning; node
chunks; the hierarchy VLR — each with a hand-countable toy example and
numbers measured from real files.

---

## COPC construction: format rules versus writer policy

**COPC is a file format and access contract, not a particular LOD algorithm.**
It says that the file is LAS 1.4 + LAZ data arranged into spatially-addressable
chunks, and that the file contains a COPC info VLR and a hierarchy VLR that
tell a reader where those chunks are. A compliant reader can then request just
the nodes that intersect a region and meet a requested resolution. COPC does
*not* require every writer to use the same root spacing, sampling rule, node
size, or tree-building implementation.

There are two layers of control:

| Concern | COPC requires | A COPC writer chooses | `PDAL writers.copc` lets you set |
|---|---|---|---|
| Root spacing / LOD ladder | Root spacing in COPC info VLR; level `d` is `spacing / 2^d` | root cube and root spacing | **No direct option** |
| Thinning | Nodes contain point records | parent-level representative-point rule | **No direct option**; `fixed_seed` only makes PDAL's selection repeatable |
| Node chunks | hierarchy entries give byte ranges and point counts | node split threshold and LAZ chunk boundaries | **No direct option** |
| Hierarchy VLR | valid COPC info + hierarchy VLRs | layout and paging of hierarchy entries | **No direct option**; PDAL writes them |
| LAS metadata | valid LAS header/VLRs | which metadata is present | scale/offset, `a_srs`, `extra_dims`, `vlrs`, `pipeline`, `pdal_metadata`, etc. |

In short: the **COPC specification** defines what must be in the file so a
client can find LOD data; the **writer implementation** decides how to build
that data. PDAL's `writers.copc` owns the four pyramid decisions above. It does
not expose `root_spacing`, `points_per_node`, `chunk_size`, or
`thinning_method` options.

`create_copc_advanced_options` is the concrete PDAL example. It controls
source density before writing (a 2 m input grid), output precision, CRS,
metadata, reproducibility and writing threads. PDAL still creates the COPC
tree automatically:

```bash
./create_copc_advanced_options
./copc_hierarchy_inspect advanced-options.copc.laz
```

To change a PDAL COPC file's pyramid indirectly, change its input: crop/tile
the cloud, pre-thin it with a PDAL filter, or add/remove points. Explicit root
spacing, exact per-node budgets, or manual hierarchy entries need a writer or
library that exposes those construction decisions (or a modified COPC writer).

### One numerical COPC example, from points to byte ranges

The four rows of that table are not four unrelated features. They are four
mechanisms that compose into one sentence:

> **The ladder** says how coarse each level is · **thinning** says which points
> land on each rung · **node chunks** say what the unit of I/O is ·
> **the hierarchy VLR** says where those bytes are.

| Mechanism | Answers the question | Where it lives in the file | Decided by |
|---|---|---|---|
| Root spacing / LOD ladder | *How coarse is level `d`?* | one `double` in the copc info VLR | writer, at creation |
| Thinning | *Which points are on rung `d`?* | the point order itself | writer, at creation |
| Node chunks | *What is the smallest thing I can read?* | LAZ chunk boundaries | writer, at creation |
| Hierarchy VLR | *Which byte range is node `(d,x,y,z)`?* | one or more hierarchy pages | writer, at creation |
| `bounds` + `resolution` | *Which of those do I actually want?* | nowhere — a runtime query | **you, per frame** |

Only the last row is yours. Everything above it was frozen when the file was
written. The rest of this section walks each mechanism twice: once on a toy
cloud small enough to count by hand, once on a real file with real numbers.

The toy: a **surface-like** cloud, a 16 m × 16 m patch measured on a regular
1 m grid, so `16 × 16 = 256` original points. To keep the sketch readable, draw
it as 2-D; a real COPC is an octree and splits along Z as well.

Suppose a writer chooses a 16 m root cube and writes this COPC info:

```text
root centre   = (8, 8, 8)
root halfsize = 8 m             -> root cube is 16 m on every side
root spacing  = 4 m
```

#### 0. Four things that get conflated: extent, density, spacing, resolution

Before any arithmetic, separate four quantities that all sound like "how big"
or "how detailed" and are not the same thing. Three of them are decided before
the file exists; only the last is yours at runtime.

```text
 YOUR INPUT CLOUD                    THE WRITER                 YOUR VIEWER
 ────────────────                    ──────────                 ───────────

 (1) EXTENT                      (3) ROOT SPACING  S        (4) RESOLUTION  r
     min/max XYZ of your             + the root cube            "how detailed
     points. "How big is                                         do I want it?"
     the world?"                  derived from (1);
        │                         NOT from (2).                (4b) BOUNDS
        └──────────────────────▶      │                            "where?"
                                      │                             │
 (2) DENSITY                          ▼                             ▼
     how many points you        spacing(d) = S / 2^d        deepest level =
     actually measured.         width(d)   = W / 2^d        ceil(log2(S / r))
     "How much real detail                                   ↑
      exists?"  ───────────────▶ determines how DEEP the     you pick only this
                                 tree actually goes
```

**(1) Extent is read off your points, never set.** `writers.copc` runs a stats
pass, takes the min/max, and builds a **cubic** root node with
`side = max(xside, yside, zside)` anchored at the data's min corner
([`Grid::Grid`](https://github.com/PDAL/PDAL/blob/master/io/private/copcwriter/Grid.cpp)).
You can see the padding in `synthetic.copc.laz`: the data's Z spans about
−20…20 m, but the root cube's Z spans −20…979 m, because the cube has to match
the 999 m X/Y extent. There is no `setExtent()`. To change the root cube you
change the input — crop, tile, or reproject it.

**(2) Density is also read off your points, and it is *not* what sets the
spacing.** Writing a 1 m grid and a 0.1 m grid over the same 1000 m square gives
you two files with the **same** root spacing and root cube, differing only in
how many levels deep the tree goes before it runs out of points. Density
controls *depth*, not the ladder. And no octree depth ever manufactures detail:
if your source is one point per 10 m, level 12 is still one point per 10 m.

**(3) Spacing is the writer's choice, derived from the extent.** For PDAL it is
not a choice at all — `spacing = rootCubeSize / 147`, hardcoded, with no option
to override. See part 1 below.

**(4) Resolution and bounds are yours, per query, and they are independent.**
`bounds` says *where*, `resolution` says *how detailed*. Neither touches the
file; both are decided against the index.

One more pair that gets conflated, and it is the most useful distinction in this
whole document — **node width and point spacing are two different lengths**, and
they halve together:

```text
level 3 node of a 1000 m cloud whose root spacing is 16 m

  +-----------------------------+
  |  .      .        .      .   |     width(3)   = 1000 / 2^3 = 125 m
  |       .     .               |       "what region of space is this node?"
  |   .             .      .    |
  |        .    .               |     spacing(3) =   16 / 2^3 =   2 m
  |   .        .         .      |       "how far apart are the points in it?"
  +-----------------------------+
```

`width / spacing` is constant at every level — here `1000/16 = 62.5` spacings
across every node, at every depth. That constant is the writer's real design
parameter (PDAL's is always 147), and it is why a node costs roughly the same
number of points wherever it sits in the tree.

> **And the idea both of those diagrams leave out:** asking for `resolution = 4`
> does **not** hand you "the level-2 nodes". It hands you **levels 0, 1 and 2
> unioned**. Every node holds real points, and a node's points are *additional*
> to its ancestors', never a replacement — see part 2 below. Read the `cumulative`
> column of `copc_hierarchy_inspect`, not the per-level one.

#### 1. Root spacing and the LOD ladder

**What `spacing` means.** It is a *distance*, in dataset units (usually metres),
and it is the nominal minimum separation between two points **inside one node**.
It is not a point count, not a level number, and not a screen resolution. A node
whose spacing is 4 m holds points that are roughly 4 m apart; nothing in that
node is denser than that.

**The ladder.** The COPC specification fixes exactly one rule: spacing halves
every level.

```text
spacing(d) = rootSpacing / 2^d
width(d)   = rootCubeSize / 2^d        <- node edge length, same halving
```

Both halve together, and that is the load-bearing consequence:

```text
width(d)             rootCubeSize
----------  =  -------------  =  constant, for every level
spacing(d)          rootSpacing
```

**Every node is the same number of spacings across, at every depth.** That
constant is the writer's only real design parameter, and it is why a node costs
roughly the same everywhere in the tree — which is what makes a per-frame point
budget possible at all ([§10](copc_worked_examples.md)).

For the toy file, `rootCubeSize / rootSpacing = 16 / 4 = 4`: every node is 4
cells on a side.

| Level | Node width | Spacing | Cells per node side | Meaning for a 1 m source grid |
|---:|---:|---:|---:|---|
| 0 | 16 m | 4 m | 4 | a very coarse overview |
| 1 | 8 m | 2 m | 4 | a medium-detail view |
| 2 | 4 m | 1 m | 4 | full source-grid detail |

The values `4, 2, 1` are **not chosen by a reader**. They are data in the file.

**Going the other way: resolution → level.** This is the arithmetic a viewer
actually runs. You want points no further apart than `r`; you need the shallowest
level whose spacing is already `<= r`:

```text
deepest level = max(0, ceil(log2(rootSpacing / r)))
```

That is literally what PDAL does — [`io/CopcReader.cpp`](https://github.com/PDAL/PDAL/blob/master/io/CopcReader.cpp)
computes an *exclusive* end depth:

```cpp
m_p->depthEnd = m_args->resolution
    ? (std::max)(1, (int)ceil(log2(m_p->copc_info.spacing / m_args->resolution)) + 1)
    : 0;                                   // 0 means "no limit, read everything"
```

Note the two clamps, because they are the two questions everybody asks:

* **Ask for something coarser than the root** (`r > rootSpacing`) and you still
  get level 0. There is nothing above the root; the ladder has a top.
* **Ask for something finer than the deepest node** and you get the whole tree.
  The ladder also has a bottom, and it is wherever the writer ran out of points,
  not wherever your arithmetic says.

**On a real file.** `lone-star.copc.laz` (a PDAL test file, 518 862 points,
2.58 MB) reports `halfsize = 20.379875`, `spacing = 0.3184355469`. Note
`40.75975 / 0.3184355469 = 128.0` exactly: that writer laid 128 sampling cells
across the root cube. Feed the formula six resolutions and check it against the
file:

| Requested `r` | `log2(spacing/r)` | `ceil` | Deepest level | Nodes touched | Points |
|---:|---:|---:|---:|---:|---:|
| 1.0 | −1.65 | −1 → clamped to 0 | 0 | 1 of 15 | 58 393 (11.25 %) |
| 0.35 | −0.14 | 0 | 0 | 1 of 15 | 58 393 (11.25 %) |
| 0.3184355469 | 0.00 | 0 | 0 | 1 of 15 | 58 393 (11.25 %) |
| 0.2 | 0.67 | 1 | 1 | 5 of 15 | 163 993 (31.61 %) |
| 0.1 | 1.67 | 2 | 2 | 15 of 15 | 518 862 (100 %) |
| 0.01 | 4.99 | 5 → tree stops at 2 | 2 | 15 of 15 | 518 862 (100 %) |

(Reproduce any row with
`./copc_hierarchy_inspect lone-star.copc.laz --bounds <full extent> --resolution <r>`.)

**How a writer picks the root spacing.** COPC does not say, so writers differ —
and you can see the difference in the files. PDAL's `writers.copc` picks it in
one line of [`io/private/copcwriter/Output.cpp`](https://github.com/PDAL/PDAL/blob/master/io/private/copcwriter/Output.cpp):

```cpp
m_copcVlr.spacing = (2.0 * m_copcVlr.halfsize) / RootCellCount;
```

with, from `Common.hpp`:

```cpp
const int MaxPointsPerNode = 100000;          // "These are hopes, not absolutes."
constexpr double Sqrt3     = 1.73205080757;
constexpr int ChildCellCount = int(128 * Sqrt3);        // 221
constexpr int RootCellCount  = int(128 * Sqrt3 / 1.5);  // 147
```

So for **any** file PDAL writes, `spacing = rootCubeSize / 147`, exactly. Check
it on two files this repo produces:

```text
synthetic.copc.laz          cube 999.000  spacing 6.795918367   999.000/147 = 6.795918367  ✓
advanced-options.copc.laz   cube 998.000  spacing 6.789115646   998.000/147 = 6.789115646  ✓
lone-star.copc.laz          cube  40.760  spacing 0.3184355469   40.760/128 = 0.3184355469  ✗ (128, not 147)
```

lone-star's LAS header carries an empty `software_id`, and its ratio is 128 —
it was written by a span-128 writer (Entwine/copc-lib lineage), not by PDAL.
**Same format, different ladder.** A reader does not care and does not need to:
it reads `spacing` out of the VLR and the arithmetic works either way. This is
exactly the "format rules versus writer policy" split from the table above,
visible in two files on your disk.

#### 2. Thinning: which points go at each level

**The rule, in one sentence:** lay a regular grid over a node at that node's
spacing, and keep **at most one point per grid cell**; everything that loses its
cell stays in a child and gets another chance one level down, on a grid twice
as fine.

This is a *Poisson-disk-ish* sample: it guarantees near-uniform density and a
minimum separation, which is exactly what "spacing" promises. Two things it is
deliberately **not**:

* Not a random percentage. `10 %` of a cloud is clumpy where the cloud is
  clumpy; a grid sample is uniform everywhere.
* Not a *derived* point. COPC never averages, snaps, or invents a point. Every
  point at every level is an **original, unmodified record** from your input —
  which is why a coarse COPC read is still a legitimate measurement, not a
  visualization artefact, and why a level-0 read is safe to run statistics on.

**PDAL's actual implementation** ([`io/private/copcwriter/Processor.cpp`](https://github.com/PDAL/PDAL/blob/master/io/private/copcwriter/Processor.cpp),
`Processor::sample()`), because the abstract rule leaves out the interesting
detail — it works **bottom-up**:

```cpp
std::shuffle(v->begin(), v->end(), *g);        // randomise, so no scan-order bias
for (PointId idx = 0; idx < v->size(); ++idx)
{
    GridKey k = m_vi.gridKey(PointRef(*v, idx));   // which cell of the PARENT grid?
    if (acceptable(k))                             // cell still free?
    {
        accepted->appendPoint(*v, idx);            // promote into the parent node
        m_vi.grid().insert(k);                     // and claim the cell
    }
    else
        rejected->appendPoint(*v, idx);            // stay in the child
}
v = rejected;
```

Points bubble *up* from children into the parent until the parent's grid is
full. `acceptable()` is the whole selection policy and it is four lines: *is
this cell already taken?*

Three consequences worth internalizing:

* **The shuffle is why `fixed_seed` exists.** Which of the candidate points in a
  cell wins is arbitrary, so two runs of `writers.copc` on identical input
  produce byte-different files. `fixed_seed=true` seeds the Mersenne twister
  with `1234` and makes the choice repeatable. It does not change the *quality*
  of the sample, only its reproducibility — which matters for regression tests
  and content-hash-addressed storage.
* **Tiny children get folded into the parent.** Also from `Common.hpp`:
  `MinimumPoints = 100`, `MinimumTotalPoints = 1500`. A child holding fewer than
  100 points is not worth a chunk of its own, so its points are moved up. This
  is why you never see 3-point nodes and why the tree stops long before the
  ladder does.
* **The grid is finer than the advertised spacing, for children.** The root node
  samples on `width/147` (= the reported spacing), children sample on
  `width/221` — a factor of 1.5 finer. `spacing` is a nominal, conservative
  promise, not a measured minimum distance.

**On the toy cloud.** Every node is 4 cells across (from part 1), so:

```text
level 0   one node, 16 m wide, grid 4x4 of 4 m cells
          -> 16 cells -> 16 points promoted                         16 points

level 1   four nodes, 8 m wide each, grid 4x4 of 2 m cells
          -> 16 cells per node, but 4 of them already hold a
             level-0 point -> 12 promoted per node, x 4 nodes       48 points

level 2   sixteen nodes, 4 m wide each, grid 4x4 of 1 m cells
          -> everything still unplaced                             192 points
          ------------------------------------------------------------------
                                                                   256 points
```

The word **additional** matters. A point selected for level 0 is not duplicated
in levels 1 and 2 — it was *moved*, not copied. Rendering levels 0–1 draws
`16 + 48 = 64` points; rendering 0–2 draws all 256. This is why a viewer can show
a coarse cloud immediately and then *append* finer points without replacing or
re-fetching anything, and why nothing ever flickers.

**On a real file.** `synthetic.copc.laz` in this repo is 1 000 000 points on a
1000 × 1000 m grid (a sine surface), written by PDAL. Predict its node
populations from the rule alone:

```text
root cube 999 m, spacing 999/147 = 6.796 m

level 0:  147 x 147 x 147 grid over the root.  The data is a SURFACE, so only
          ~147 x 147 = 21 609 XY columns are occupied.  Each column's z may
          straddle two z cells where the surface is steep.
          prediction: a bit over 21 609.            actual: 27 279   (1.26x)

level 1:  four occupied nodes, each 499.5 m wide, grid 221 cells across.
          221 x 221 = 48 841 columns per node, minus the handful the parent
          already claimed, plus z-straddling.
          prediction: ~49 000 per node.             actual: 51 043 avg  ✓

level 2:  sixteen nodes; everything left over.      actual: 768 549
```

```text
 level   nodes      points    spacing   avg pts/node   cumulative pts
 -----   -----   ---------   --------   ------------   --------------
     0       1       27279     6.7959          27279            27279
     1       4      204172     3.3980          51043           231451
     2      16      768549     1.6990          48034          1000000
```

The `avg pts/node` column staying flat at ~48–51 k across levels 1 and 2 is the
`width/spacing = constant` invariant from part 1 showing up as a measurement. That
flatness is the entire reason a viewer can budget a frame by counting *nodes*.

#### 3. Node chunks: compressed payloads, not one monolithic point block

**One node = one LAZ chunk = one contiguous byte range = one HTTP request.**
That equality is the only thing COPC adds to LAZ's physical layout, and it is
what makes the node the **atomic unit of I/O**.

Ordinary LAZ already chunks: it cuts the point stream every ~50 000 points and
starts a fresh arithmetic coder, so any chunk decompresses independently, and a
chunk table at the end of the file lists each chunk's offset. COPC keeps all of
that and changes one thing: it uses LAZ's **variable-size** chunking so that
chunk boundaries land exactly on node boundaries. Nothing about the codec
changes; a plain LAZ reader still reads the file front to back and sees a normal
(oddly ordered) point cloud.

For the toy cloud, level 0 is one node and one chunk of 16 points; level 1 has
up to eight children, of which only four XY children are populated for this flat
surface:

```text
node (0,0,0,0)       -> LAZ chunk: 16 points
node (1,0,0,*)       -> LAZ chunk: 12 points
node (1,1,0,*)       -> LAZ chunk: 12 points
node (1,0,1,*)       -> LAZ chunk: 12 points
node (1,1,1,*)       -> LAZ chunk: 12 points
... level-2 nodes    -> LAZ chunks containing the remaining 192 points
```

`*` means the Z child depends on where the surface falls in the root cube. For a
non-flat aerial cloud many more of the eight 3-D children are populated.

**Chunks are not uniform, and the spec does not pretend otherwise.** Here is the
real node list of lone-star, straight out of `copc_hierarchy_inspect --tree`:

```text
 key (l-x-y-z)      points     bytes    bytes/point
0-0-0-0              58393    431475       7.39
  1-0-0-0            32909    159695       4.85
  1-0-1-0             8387     64716       7.72
  1-1-0-0            30657    146833       4.79
  1-1-1-0            33647    259054       7.70
    2-0-2-0            212      1438       6.78
    2-1-2-0          42862    202277       4.72
    2-1-2-1          11135     50154       4.50
    2-1-3-0          30018    141467       4.71
    2-2-2-0         126731    585499       4.62   <-- 1.27x MaxPointsPerNode
    2-2-2-1          58730    263163       4.48
    2-2-3-0          54282    251691       4.64
    2-3-2-0           5885     27975       4.75
    2-3-3-0          24696    116124       4.70
    2-3-3-1            318      1733       5.45
```

Read three things off that table:

* **Point counts span 318 to 126 731 — a 400× range.** `MaxPointsPerNode =
  100000` is commented in PDAL's source as *"These are hopes, not absolutes"*,
  and node `2-2-2-0` is the proof: it is a leaf, so every point that never got
  promoted had to land somewhere. The budget constrains *interior* nodes, which
  are grid-sampled; leaves take the remainder.
* **Compression is 4.5–7.7 bytes per point** for a 30-byte PDRF-6 record —
  4–6× — and it is *better in dense nodes*, because LAZ predicts each point from
  its neighbours and dense nodes have closer neighbours. Coarse nodes compress
  worse per point. Budget in bytes, not in points.
* **Every node's byte size is in the index**, so you can total the cost of a
  query exactly before issuing it. That is part 5 below.

**The over-fetch tax.** Because the node is atomic, a reader always decompresses
*whole* nodes and only then crops to your box. The gap can be large:

```text
$ ./copc_hierarchy_inspect lone-star.copc.laz \
      --bounds 515399,515409,4918370,4918381 --resolution 0.1
  Points in those nodes : 362682          <- upper bound, what gets decompressed
  Bytes read            : 1936714         <- exact, what leaves the disk/network

$ pdal translate lone-star.copc.laz q.las \
      --readers.copc.bounds="([515399,515409],[4918370,4918381])" \
      --readers.copc.resolution=0.1
  num_points            : 37058           <- what you actually get back
```

**362 682 decompressed to hand you 37 058** — a ~10× over-fetch, and almost all
of it is the root node, which spans the entire dataset and is unavoidably
touched by every query. The number that is *exact*, and the one that actually
costs you time and money, is **bytes read**. Cost a query in bytes.

Over-fetch shrinks as your box grows and as you go deeper (deeper nodes are
smaller, so they straddle your box less). It is worst for a tiny box at a coarse
resolution — which, conveniently, is also the cheapest case in absolute terms.

#### 4. Hierarchy VLR: the directory from octree keys to chunks

The hierarchy is the file's directory, and it is a flat array of fixed 32-byte
entries:

| Field | Type | Bytes | Meaning |
|---|---|---:|---|
| `key.level` | `int32` | 4 | octree depth, 0 = root |
| `key.x` | `int32` | 4 | node index along X at that level, `0 .. 2^level - 1` |
| `key.y` | `int32` | 4 | … Y |
| `key.z` | `int32` | 4 | … Z |
| `offset` | `uint64` | 8 | absolute byte offset of this node's LAZ chunk |
| `byteSize` | `int32` | 4 | compressed size of that chunk |
| `pointCount` | `int32` | 4 | `> 0` node · `0` empty node · `-1` **this is a child page** |

**What is conspicuously absent: the node's bounding box.** It is not stored,
because it is pure arithmetic from the key and the root cube:

```cpp
double size = rootSize / (1 << key.level);
minx = rootMinX + key.x * size;   maxx = minx + size;
// same for y and z
```

That omission is why the index is so small. lone-star has 15 nodes and its
hierarchy is `root_hier_size = 480` bytes — exactly `15 × 32`. `synthetic.copc.laz`
has 21 nodes and a 672-byte hierarchy — exactly `21 × 32`. A hierarchy with a
million nodes is 32 MB, which is still a rounding error against a 500 GB cloud,
and you never read all of it anyway, because of paging.

**Paging, and why the index itself is lazy.** `pointCount == -1` means the entry
is not a node at all: its `offset`/`byteSize` locate **another hierarchy page**
rooted at that key. A client that only ever looks at the north-east corner
follows only the pages down that corner and never downloads the rest of the
directory.

```text
root page  (offset from copc info VLR)
  ├── (1,0,0,0)  pointCount   4211   -> a real chunk, read it
  ├── (1,1,0,0)  pointCount     -1   -> a child PAGE: another range request,
  │                                     32 bytes x however many entries it has
  ├── (1,0,1,0)  pointCount      0   -> node exists in the key space but is
  │                                     empty; do not request it, do not descend
  └── ...
```

`pointCount == 0` is the third case and it is not the same as absent: it says
"this key is known and has no points". Treat it as a leaf and stop.

**The full client walk**, which is what `readers.copc` and
[src/copc_index.hpp](../src/copc_index.hpp) both implement:

```text
1. range-read bytes 0..375            -> LAS header: extent, count, EVLR offset
2. range-read bytes 429..589          -> copc info VLR: root cube, spacing,
                                         root hierarchy page offset + size
3. range-read the root hierarchy page -> a std::map<VoxelKey, Entry>
4. queue <- root key
   while queue not empty:
       key <- pop
       if spacing(key.level) < requested resolution:   drop subtree   (depth cut)
       if bounds(key) does not intersect query box:    drop subtree   (space cut)
       e <- index[key]
       if e.pointCount == -1:   range-read that page, merge into index, re-queue key
       if e.pointCount ==  0:   nothing here, stop
       else:                    range-read [e.offset, e.offset+e.byteSize)
                                LAZ-decompress, append to the PointView
       push the 8 children of key
```

Steps 1–3 are **three range requests and under 1 kB** on a file of any size, and
after them you know the extent, the point count, the LOD ladder, and the exact
byte cost of every node you might want. Everything after that is `bytes read`
you chose deliberately.

#### 5. All four at once: one query, traced node by node

Take lone-star, ask for the north-east corner at 0.1 m:

```text
bounds     = ([515399, 515409], [4918370, 4918381])
resolution = 0.1   ->  deepest level = ceil(log2(0.31844/0.1)) = 2
```

Walk all 15 nodes, decide each one by key arithmetic alone:

| Node | X range | Y range | Verdict | Points | Bytes |
|---|---|---|---|---:|---:|
| `0-0-0-0` | 515368.6–515409.4 | 4918340.4–4918381.1 | **read** | 58 393 | 431 475 |
| `1-0-0-0` | 515368.6–515389.0 | 4918340.4–4918360.7 | cull: X | – | – |
| `1-0-1-0` | 515368.6–515389.0 | 4918360.7–4918381.1 | cull: X | – | – |
| `1-1-0-0` | 515389.0–515409.4 | 4918340.4–4918360.7 | cull: Y | – | – |
| `1-1-1-0` | 515389.0–515409.4 | 4918360.7–4918381.1 | **read** | 33 647 | 259 054 |
| `2-0-2-0` | 515368.6–515378.8 | 4918360.7–4918370.9 | cull: X | – | – |
| `2-1-2-0` | 515378.8–515389.0 | 4918360.7–4918370.9 | cull: X | – | – |
| `2-1-2-1` | 515378.8–515389.0 | 4918360.7–4918370.9 | cull: X | – | – |
| `2-1-3-0` | 515378.8–515389.0 | 4918370.9–4918381.1 | cull: X | – | – |
| `2-2-2-0` | 515389.0–515399.2 | 4918360.7–4918370.9 | **read** (0.2 m overlap) | 126 731 | 585 499 |
| `2-2-2-1` | 515389.0–515399.2 | 4918360.7–4918370.9 | **read** | 58 730 | 263 163 |
| `2-2-3-0` | 515389.0–515399.2 | 4918370.9–4918381.1 | **read** | 54 282 | 251 691 |
| `2-3-2-0` | 515399.2–515409.4 | 4918360.7–4918370.9 | **read** | 5 885 | 27 975 |
| `2-3-3-0` | 515399.2–515409.4 | 4918370.9–4918381.1 | **read** | 24 696 | 116 124 |
| `2-3-3-1` | 515399.2–515409.4 | 4918370.9–4918381.1 | **read** | 318 | 1 733 |
| | | | **8 read, 7 culled** | **362 682** | **1 936 714** |

which is exactly what the tool reports:

```text
Nodes touched        : 8 of 15
  culled by bounds   : 7
  culled by depth    : 0
Points in those nodes: 362682 of 518862   (69.90 %)
Bytes read           : 1936714 of 2705193   (71.59 %)
Range requests       : 10   (header + root hierarchy page + one per node)
```

Three details in that trace are worth pulling out:

* **`2-2-2-0` and friends are read for a 0.2 m sliver of overlap.** Node AABB
  testing is conservative by construction; a box that clips a node's edge pays
  for the whole node. Snapping your query box outward to node boundaries costs
  you nothing extra and tells you the truth about what you are about to fetch.
* **The root is always read.** It spans the dataset, so it intersects every
  query. On a big file that is your fixed per-query floor.
* **Now move only `resolution` and watch the other knob work**, same file, whole
  extent this time so no cropping muddies the counts:

```text
resolution 0.35  ->  level 0    58393 pts (11.25 %)    431475 bytes (15.95 %)   3 requests
resolution 0.2   ->  level 1   163993 pts (31.61 %)   1061773 bytes (39.25 %)   7 requests
resolution 0.1   ->  level 2   518862 pts (100.0 %)   2703294 bytes (99.93 %)  17 requests
no resolution    ->  all       518862 pts (100.0 %)   2703294 bytes (99.93 %)  17 requests
```

Two knobs, two independent prunings, both decided from a sub-1 kB index before
any point byte is touched:

| Knob | Prunes | Mechanism | Cost of getting it wrong |
|---|---|---|---|
| `bounds` | **in space** | node AABB (computed from the key) vs. the query box | over-fetch: whole nodes decompressed, then cropped |
| `resolution` | **in depth** | `rootSpacing/2^level` vs. requested spacing | one level too deep ≈ 4× the bytes on a 2.5-D surface |

That last figure is the one to remember for [§10](copc_worked_examples.md): for a surface-like cloud, point
count scales as `spacing^-2`, so **each extra level costs about 4×**. Getting the
screen-space-error test right is worth more than any other optimization in the
viewer.

> **If one question brought you here — "where do the levels of detail come
> from?" — the short answer is: from the file.** A COPC file stores one number,
> the root node spacing, and level *d* has spacing `rootSpacing / 2^d`. The
> writer fixed that ladder when the file was created. PDAL does not invent it,
> VTK does not invent it, and your viewer does not invent it. The only thing a
> viewer chooses is **which rungs to fetch**, and [§10](copc_worked_examples.md) shows that choice being
> derived from nothing but the camera. Run `copc_hierarchy_inspect` on any COPC
> file to see the ladder printed out ([§8](copc_worked_examples.md)).

