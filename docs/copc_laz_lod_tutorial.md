# LAZ, COPC, PDAL, octrees, LOD, and the VTK camera

How a 500 GB point cloud becomes 2 million points on screen, and which layer of
the stack is responsible for each step.

Code in this repo:

| Target | File | What it shows |
|---|---|---|
| `create_copc_in_memory` | [src/create_copc_in_memory.cpp](../src/create_copc_in_memory.cpp) | Build a `PointView` in memory, write it as COPC, read one bounds box back |
| `create_copc_advanced_options` | [src/create_copc_advanced_options.cpp](../src/create_copc_advanced_options.cpp) | PDAL's real writer/header/metadata controls, contrasted with automatic COPC-pyramid construction |
| `copc_lod_queries` | [src/copc_lod_queries.cpp](../src/copc_lod_queries.cpp) | Same bounds, five different `resolution` values — the LOD pyramid made numeric |
| `camera_lod_COCP` | [src/camera_lod_COCP.cpp](../src/camera_lod_COCP.cpp) | The VTK side: camera, focal point, frustum planes, projected pixel size → LOD level |
| `copc_hierarchy_inspect` | [src/copc_hierarchy_inspect.cpp](../src/copc_hierarchy_inspect.cpp) | **No dependencies.** Reads the octree index out of the file and costs a query before running it — [§8](copc_worked_examples.md) |
| *(header)* | [src/copc_index.hpp](../src/copc_index.hpp) | The COPC index parser and query-costing arithmetic, shared by the inspector and the streaming viewer |
| `copc_partial_load_vtk` | [src/copc_partial_load_vtk.cpp](../src/copc_partial_load_vtk.cpp) | Load one box at one LOD into a `vtkPolyData` and render it — [§9](copc_worked_examples.md) |
| `copc_camera_streaming` | [src/copc_camera_streaming.cpp](../src/copc_camera_streaming.cpp) | The closed loop: the camera derives both the bounds and the resolution — [§10](copc_worked_examples.md) |
| *(python)* | [python/copc_hierarchy_inspect.py](../python/copc_hierarchy_inspect.py) | Same inspector in Python, **and it works over HTTP** on a remote file — [§8a](copc_worked_examples.md) |
| *(python)* | [python/copc_partial_load_vtk.py](../python/copc_partial_load_vtk.py) | Partial load + VTK in Python: URLs, predicted-vs-actual cost, offscreen PNGs — [§9a](copc_worked_examples.md) |
| *(python)* | [python/copc_camera_streaming.py](../python/copc_camera_streaming.py) | The streaming loop in Python, with the budget enforced from the index — [§10a](copc_worked_examples.md) |

The PDAL targets need `-DUSE_PDAL=ON`; PDAL must be installed *before* VTK is
configured (see [README](../README.md)). `camera_lod_COCP` is pure VTK.
`copc_hierarchy_inspect` needs nothing at all — `g++ -std=c++17 -O2 -o
copc_hierarchy_inspect src/copc_hierarchy_inspect.cpp` and it runs.

---

## Read it in this order

| | Chapter | What it answers |
|---|---|---|
| 1 | **[The COPC file format](copc_format.md)** | What is physically in the file — LAS container, LAZ codec, the two VLRs, the byte map, the spec's constraints, and what COPC deliberately does *not* do. Plus EPT/Potree/COPC side by side. |
| 2 | **[How COPC builds levels of detail](copc_lod_mechanisms.md)** | Extent vs. density vs. spacing vs. resolution; the LOD ladder; thinning; node chunks; the hierarchy VLR — with a toy example you can count by hand and real measured numbers. **Start here if the question is "where do the levels come from?"** |
| 3 | **[PDAL](copc_pdal.md)** | The pipeline model, `bounds` and `resolution`, every `readers.copc` option and which ones actually save I/O, and the write path. |
| 4 | **[The VTK camera and the streaming loop](copc_vtk_camera.md)** | Position, focal point, frustum planes, screen-space error, and the loop that turns them into one query. |
| 5 | **[Worked examples](copc_worked_examples.md)** | Six runnable programs, C++ and Python, local and over HTTP, each with its real output. |

If you only run one thing, run
[`copc_hierarchy_inspect`](copc_worked_examples.md): it needs no dependencies at
all and it prints the LOD ladder straight out of any COPC file.

---

## The whole picture in one diagram

```
 DISK / HTTP                    PDAL                          VTK
 ───────────                    ────                          ───

 big.copc.laz
 ┌──────────────────┐
 │ LAS header       │
 │ COPC info VLR    │──┐
 │ COPC hierarchy   │  │  "which byte range
 │   VLR (octree    │  │   holds node 3-4-2-1?"
 │   index)         │  │
 ├──────────────────┤  │       readers.copc
 │ chunk 0  (node   │  │       ┌─────────────────┐
 │          0-0-0-0)│  └──────▶│ walk hierarchy  │
 │ chunk 1  (node   │          │ cull by bounds  │      vtkPoints
 │          1-0-0-0)│◀─ range ─│ stop at depth   │      vtkPolyData
 │ chunk 2  ...     │  reads   │ (resolution)    │──▶   vtkPolyDataMapper
 │ ...              │          │ LAZ-decompress  │      vtkActor
 │ chunk N          │          └─────────────────┘         │
 └──────────────────┘                  ▲                   │
                                       │                   ▼
                              bounds + resolution      vtkRenderer
                                       │                   │
                                       └───────────────────┘
                                   camera position / focal point /
                                   frustum planes / screen-space error
```

The loop is closed: **the camera decides what to ask PDAL for, PDAL decides
which bytes to read, VTK draws the result, and drawing moves the camera again.**

---

## Glossary

| Term | One line |
|---|---|
| **LAS** | Uncompressed binary point format; header + VLRs + fixed-stride records |
| **LAZ** | Losslessly compressed LAS; chunked (~50 k points) and independently decompressible per chunk |
| **COPC** | LAZ whose chunks are octree nodes, plus VLRs indexing them → spatial + LOD access by byte range |
| **VoxelKey** | `(level, x, y, z)`; identifies a node, and its bounds are arithmetic from the root cube |
| **Node spacing** | Nominal minimum distance between points *within one node* at that level; halves each level down |
| **Node width** | Edge length of the node's cube, `rootCubeSize / 2^level`. Halves alongside spacing — `width/spacing` is constant at every depth |
| **Root cube** | Cubic bounding node, `side = max(xside, yside, zside)`, anchored at the data's min corner. Cubic even when the data is not |
| **copc info VLR** | 160 bytes at fixed offset 429: root cube, spacing, root hierarchy page location, GPS time range |
| **Hierarchy entry** | 32 bytes: `VoxelKey` + offset + byteSize + pointCount. `pointCount` `0` = empty node, `-1` = child page |
| **Hierarchy page** | A contiguous run of entries, fetched on demand — how the *index itself* stays lazy |
| **Chunk** | One node's LAZ payload; the atomic unit of I/O. Variable size; its bytes and point count are in the index |
| **Over-fetch** | Whole nodes are decompressed then cropped to `bounds`, so returned points < points-in-nodes. Budget in **bytes**, not points |
| **`fixed_seed`** | `writers.copc` option making the random tie-break inside a sample cell repeatable; does not change sample quality |
| **PDRF** | Point Data Record Format. COPC permits only 6, 7 and 8 (LAS 1.4, GPS time mandatory) |
| **EPT / Entwine** | The ancestor format: one file per node + JSON hierarchy |
| **Potree** | Web point-cloud viewer and its own octree format; reads COPC directly in 2.0 |
| **PDAL** | Pipeline library of reader/filter/writer stages over `PointTable`/`PointView` |
| **`bounds`** | PDAL spatial filter `([xmin,xmax],[ymin,ymax][,[zmin,zmax]])`; prunes the traversal in space |
| **`resolution`** | PDAL COPC option; target point spacing — prunes the traversal in depth |
| **Frustum** | Truncated pyramid of visible space; 6 inward-facing planes from `vtkCamera::GetFrustumPlanes` |
| **Focal point** | Camera aim point *and* trackball rotation pivot |
| **Screen-space error** | Node spacing projected to pixels; the standard "do I need to refine?" test |
| **LOD** | Drawing coarser data when it occupies few pixels — here, choosing an octree depth |

## Further reading

* COPC specification — <https://copc.io/> (and the normative text at
  <https://github.com/copcio/copcio.github.io>)
* LAS 1.4 R15 specification (ASPRS) — the base container COPC constrains
* `copc-lib` — an independent C++/Python COPC reader/writer, useful for
  cross-checking a file written by PDAL: <https://github.com/RockRobotic/copc-lib>
* lazperf — the LAZ codec and the `copc_info_vlr` struct quoted above:
  <https://github.com/hobuinc/laz-perf>
* PDAL's COPC writer internals (spacing, sampling, node budgets) —
  `io/private/copcwriter/` in the PDAL tree: `Grid.cpp`, `Processor.cpp`,
  `Common.hpp`, `Output.cpp`
* PDAL `readers.copc` — <https://pdal.io/en/latest/stages/readers.copc.html>
* PDAL `writers.copc` — <https://pdal.io/en/latest/stages/writers.copc.html>
* Entwine / EPT — <https://entwine.io/>
* Potree — <https://github.com/potree/potree>
* In this repo: [frustum.md](frustum.md), [camera_position.md](camera_position.md),
  [octree.md](octree.md), [culling.md](culling.md),
  [point_visibility_in_camera_frustum.md](point_visibility_in_camera_frustum.md),
  [actor_multiple_levels_of_detail.md](actor_multiple_levels_of_detail.md)
