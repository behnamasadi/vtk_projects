# Autzen Stadium — evaluation data

Standard PDAL test data for LAS / LAZ / COPC evaluation. Upstream repo:
https://github.com/PDAL/data/tree/main/autzen

> The `*.laz` files are **not committed** to this repo (see `.gitignore`).
> Run `./fetch_autzen.sh` in this directory to download them locally.
> Paths below assume this directory as cwd: `data/autzen/`.

## Files

| File | Points | Size | Source | Use for |
|---|---|---|---|---|
| `autzen.laz` | 10,653,336 | ~56 MB | `https://s3.amazonaws.com/hobu-lidar/autzen.laz` | Original 2010 Autzen Stadium, unclassified. Coordinates in **US survey feet**. Full LAS dimensions (Intensity, RGB, PointSourceId, UserData). Baseline for read/decode, color mapping, height coloring. |
| `autzen-classified.copc.laz` | 10,653,336 | ~81 MB | `https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz` | Same cloud, manually classified (2021, 21 classes) + **COPC octree**. Best file for LOD / partial-load / camera-streaming evaluation (`copc_partial_load_vtk`, `copc_camera_streaming`, `copc_hierarchy_inspect`). |
| `stadium-utm.laz` | subset | small | Git LFS: `PDAL/data:autzen/stadium-utm.laz` | Small UTM clipping around the stadium. Fast smoke test. Requires `git lfs`. |
| `autzen-hole-utm.laz` | subset | small | Git LFS: `PDAL/data:autzen/autzen-hole-utm.laz` | Clipping with a hole. Good for boundary / missing-data cases. Requires `git lfs`. |
| `autzen-classified.laz` | 10,653,336 | ~? | Git LFS: `PDAL/data:autzen/autzen-classified.laz` | Classified but **not** COPC. Compare plain LAZ vs COPC I/O. Requires `git lfs`. |

S3 hosts the two full files directly (no LFS client needed). The small
clips live only in the GitHub repo behind Git LFS.

## Fetch

```bash
cd data/autzen
./fetch_autzen.sh          # both S3 files (~137 MB total)
./fetch_autzen.sh --list   # show URLs + expected sizes without downloading
```

Small LFS clips (needs `git lfs`):

```bash
# shallow, no full clone:
git lfs install
mkdir -p /tmp/autzen-upstream && cd /tmp/autzen-upstream
git init && git remote add origin https://github.com/PDAL/data.git
git lfs install --local
git sparse-checkout set autzen
GIT_LFS_SKIP_SMUDGE=1 git pull --depth 1 origin main
git lfs pull --include "autzen/stadium-utm.laz,autzen/autzen-hole-utm.laz"
```

## Classifications (`autzen-classified.*`)

| Value | Description |
|---|---|
| 2 | Ground |
| 5 | Vegetation |
| 6 | Building |
| 9 | Water |
| 15 | Transmission Tower |
| 17 | Bridge Deck |
| 19 | Overhead Structure |
| 64 | Wire |
| 65 | Car |
| 66 | Truck |
| 67 | Boat |
| 68 | Barrier |
| 69 | Railroad Car |
| 70 | Elevated Walkway |
| 71 | Covered Walkway |
| 72 | Pier/Dock |
| 73 | Fence |
| 74 | Tower |
| 75 | Crane |
| 76 | Silo/Storage Tank |
| 77 | Bridge Structure |

## Quick evaluation with this repo

No PDAL/VTK needed — read the LOD ladder straight out of the file:

```bash
g++ -std=c++17 -O2 -o /tmp/copc_hierarchy_inspect ../../src/copc_hierarchy_inspect.cpp
/tmp/copc_hierarchy_inspect autzen-classified.copc.laz
# remote variant, no download:
python3 ../../python/copc_hierarchy_inspect.py https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz
```

With PDAL (`-DUSE_PDAL=ON` targets):

```bash
pdal info --summary autzen.laz
pdal info --summary autzen-classified.copc.laz
# one COPC node at one LOD:
pdal translate autzen-classified.copc.laz stadium-out.laz \
  --readers.copc.resolution=2.3 --readers.copc.bounds="([488500,489500],[589500,590500])"
./build/copc_partial_load_vtk autzen-classified.copc.laz
./build/copc_camera_streaming autzen-classified.copc.laz
```

## Why this data

- Mix of structures (stadium, baseball park, bridges, wires, cars) — good for classification color maps and filtering.
- Full dimension set (XYZ, Intensity, RGB, PointSourceId, UserData, Classification in the classified files).
- Awkward CRS (US-ft) in the original — good reprojection test; UTM clips sidestep it for quick checks.
- 10.6 M points: big enough that brute-force full loads hurt, small enough to download (~1 min).
- COPC variant has a real 6-level octree (root spacing ~36.4) — see `docs/copc_worked_examples.md` for measured node/point counts.

## Credits

Data: Aaron Reyna, Watershed Sciences, Inc. (2010, via libLAS testing).
Classification: Max Sampson, Hobu, Inc. (2021). Upstream README and viewer:
http://autzen.entwine.io
