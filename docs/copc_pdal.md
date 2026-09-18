# PDAL — reading and writing COPC

> [index](copc_laz_lod_tutorial.md) · [1. Format](copc_format.md) · [2. LOD mechanisms](copc_lod_mechanisms.md) · **3. PDAL** · [4. VTK camera](copc_vtk_camera.md) · [5. Worked examples](copc_worked_examples.md)

The pipeline model, what `readers.copc` does with `bounds` and `resolution`,
every reader option and which ones actually save I/O, and the write path.

---

## 4. PDAL — the abstraction over all of it

PDAL is a pipeline library: **stages** connected into a DAG, moving points
through a **PointTable**.

```
readers.copc  ──▶  filters.range  ──▶  filters.reprojection  ──▶  writers.las
  (source)          (classify=2)         (EPSG:25832)             (sink)
```

Three objects to know:

| Object | Role |
|---|---|
| `PointTable` | Owns the buffer and the `PointLayout` — which dimensions exist and their types |
| `PointLayout` | Registry of `Dimension::Id::X`, `Y`, `Z`, `Intensity`, `Classification`, … plus user dimensions |
| `PointView` | A *view*: a list of point ids into a table. `execute()` returns a `PointViewSet` |

Points are accessed by (dimension, id), not as structs:

```cpp
double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
view->setField(pdal::Dimension::Id::Z, i, 0.0);
```

This columnar model is why PDAL can add a dimension mid-pipeline without
touching the reader, and why the same filter code works on LAS, PLY, E57 or a
database.

### What `readers.copc` actually does with your options

```cpp
options.add("filename",   "big.copc.laz");
options.add("bounds",     "([100,200],[300,400])");   // X range, Y range (Z free)
options.add("resolution", 5.0);                        // metres between points
```

Inside, roughly:

```
read header + copc info VLR
read root hierarchy page
queue ← root node
while queue not empty:
    node ← pop
    if node.bbox does not intersect `bounds`:      skip subtree
    if spacing(node.level) < requested resolution: skip subtree   ← the LOD cut
    if node.pointCount == -1:  fetch that child hierarchy page, re-queue
    else:
        range-read [node.offset, node.offset+node.byteSize)
        LAZ-decompress into the PointView
        push node's 8 children
```

Two independent prunings — **`bounds` prunes in space, `resolution` prunes in
depth** — and both happen *before* any decompression. That is the whole win.

Note `resolution` is a *point spacing in dataset units*, not a level number and
not a point count. The traversal stops descending once a node is already finer
than you asked for, so the returned count is "whatever the octree had at or
above that spacing" — which is why the numbers in `copc_lod_queries` are not
round.

### Every `readers.copc` option, and which ones save I/O

`bounds` and `resolution` get all the attention because they are the two the
*index* can answer. The full list (from `CopcReader::addArgs`) is worth knowing,
because half of it changes what you pay and half of it only changes what you
get:

| Option | Default | What it does | Saves I/O? |
|---|---|---|---|
| `bounds` | — | `([xmin,xmax],[ymin,ymax][,[zmin,zmax]])`; prunes the traversal **in space** | **yes** — whole subtrees never requested |
| `resolution` | 0 (= no limit) | target point spacing; prunes the traversal **in depth** | **yes** — whole levels never requested |
| `polygon` | — | GeoJSON/WKT polygon(s); node AABBs are tested against the polygon's envelope, then points are cropped exactly | **partly** — culls by envelope, crops by polygon |
| `ogr` | — | same, but the geometry comes from an OGR datasource (a shapefile, a PostGIS query) | partly |
| `requests` / `threads` | 2 local, **10 remote** | worker threads issuing chunk requests | no, but it is the single biggest wall-clock knob over HTTP |
| `keep_alive` | 10 | chunks held in memory while working | no — RAM ceiling, not I/O |
| `nosrs` | false | skip reading/processing the file's SRS | no |
| `srs_vlr_order` | — | preference among `wkt1`, `wkt2`, `projjson` when several CRS VLRs exist | no |
| `vlr` | true | copy the file's VLRs into PDAL metadata | no |
| `fix_dims` | true | rewrite invalid extra-dimension names (`my dim` → `my_dim`) | no |

Two things to take from that table:

* **Only `bounds`, `resolution` and (partly) `polygon`/`ogr` touch the amount of
  data read.** Everything downstream of the reader — `filters.range`,
  `filters.expression`, a classification test, an intensity threshold — runs
  *after* decompression. They reduce what reaches your renderer; they do not
  reduce what leaves the disk. Reading "just the ground points" of a region
  costs exactly the same bytes as reading the whole region.
* **`requests` defaults to 10 for remote files and 2 for local ones**, and the
  PDAL source comments say why: local reads are dominated by whatever you do
  with the points afterwards, remote reads are dominated by round-trip latency.
  If you are streaming from S3 and it feels slow, this is the first knob.

### `copc_lod_queries` makes this visible

[src/copc_lod_queries.cpp](../src/copc_lod_queries.cpp) writes a 1000×1000 grid
(1 M points, 1 m spacing, `z = 20·sin(0.02x)·cos(0.02y)`), converts CSV → COPC
via `readers.text → writers.copc`, then queries the *same* 100×100 m box five
times, changing only `resolution`:

```
resolution 20 m ──┐
resolution 10 m   │   identical bounds ([100,200],[300,400])
resolution  5 m   ├── only the depth cut moves
resolution  2 m   │
no resolution   ──┘   full detail
```

Expect roughly a 4× rise in point count per halving of the resolution (area
scales as spacing⁻² for a 2.5 D surface). Run it and read the five numbers — that
table *is* the LOD pyramid.

### `create_copc_in_memory` — the write path

[src/create_copc_in_memory.cpp](../src/create_copc_in_memory.cpp) shows the
other direction: you already have points in memory and want COPC out.

The trick is that a writer needs an upstream *Stage*, not a `PointView`. The
adapter is `BufferReader`:

```cpp
pdal::BufferReader buffer;
buffer.addView(view);          // wrap the in-memory view as a source stage
writer->setInput(buffer);
writer->prepare(table);
writer->execute(table);
```

`writers.copc` does the expensive part — it builds the octree, subsamples each
node to its spacing, orders the chunks, and writes the hierarchy VLRs. Note this
example's Z is constant 0, so the octree degenerates into a quadtree; make Z
vary if you want to see real 3 D node splitting.

