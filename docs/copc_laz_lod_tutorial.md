# COPC, LAZ, LOD, and camera tutorial

This project has two PDAL examples:

- `create_copc_in_memory`: creates an in-memory grid and writes `test.copc.laz`.
- `copc_lod_queries`: creates a CSV-backed COPC file and queries one area at several resolutions.

Enable them with `-DUSE_PDAL=ON` after PDAL and VTK are installed.

## LAS, LAZ, and COPC

LAS is a binary point-cloud format containing dimensions such as X, Y, Z,
intensity, classification, return number, and GPS time. LAZ is the losslessly
compressed form of LAS. A plain LAZ file is compact, but does not by itself
provide efficient spatial range access.

COPC means Cloud Optimized Point Cloud. It is a LAZ file organized into a
spatial hierarchy of chunks. Its hierarchy metadata points to byte ranges for
nodes, so readers can request only the data needed for an area and detail
level. The usual filename extension is `.copc.laz`.

## LOD and range queries

LOD is a representation of the same cloud with less detail. Coarse LODs show
fewer points when the cloud occupies few screen pixels; finer LODs add points
when the camera is closer.

A PDAL bounds range query such as below selects X from 100 to 200 and Y from
300 to 400, without constraining Z:

```text
([100,200],[300,400])
```

For COPC, PDAL traverses the hierarchy and reads only the intersecting chunks.
The `resolution` option requests a desired point spacing: larger values give a
coarser result. Omit it for full available detail in the selected bounds.

```text
readers.copc --bounds / resolution--> filtered PointView --> VTK rendering
```

## PDAL, Potree, and VTK octrees

| Component | Purpose | Hierarchy |
|---|---|---|
| PDAL | Read, write, filter, and transform clouds | COPC hierarchy when reading/writing COPC |
| Potree | Stream and render large clouds on the web | Potree converted octree |
| VTK | Visualize and analyze data in C++ | In-memory locators such as `vtkOctreePointLocator` |

COPC is an exchange and storage format. Potree uses its own converted dataset
for browser streaming. VTK octree locators are typically in-memory search
structures for tasks such as nearest-point and radius queries; they do not
make arbitrary VTK data into a COPC file.

## VTK camera pivot

`vtkInteractorStyleTrackballCamera` rotates around the active camera focal
point. The camera position is the eye, the focal point is the rotation pivot
and aim point, and view-up controls image orientation.

`camera_lod_COCP` makes this visible:

- Red sphere and label: current focal point / rotation pivot.
- Yellow line: camera position to focal point.
- World axes: origin reference.
- During a pan, the marker and line follow the new focal point.

The demo maps projected cloud size to a conceptual COPC LOD. This is viewer
policy, not a COPC specification rule. A production viewer can use the chosen
LOD and bounds to make a PDAL COPC request, then display that subset in VTK.
