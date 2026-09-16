#!/usr/bin/env python3
"""
copc_partial_load_vtk.py -- load ONLY part of a COPC point cloud and render it.

The Python twin of src/copc_partial_load_vtk.cpp, with three additions that the
C++ version cannot easily offer:

    * it works on a URL as well as a local file
    * it predicts the cost of the query BEFORE running it, by reusing
      copc_hierarchy_inspect.py, then reports predicted vs actual
    * it renders offscreen to a PNG, so it is useful over SSH and in CI

--------------------------------------------------------------------------
THE POINT

A COPC file is an octree. Asking for a box at a level of detail means the
reader walks that octree, discards every subtree that misses your box or is
finer than you asked for, and decompresses only what is left. Nothing else is
read, decompressed or allocated. Give it a 200 m box out of a 4 km cloud at
coarse resolution and you get a few tens of thousands of points for a few
hundred KB of I/O -- whether the file is 80 MB or 800 GB.

The two knobs, and they are independent:

    bounds      prunes in SPACE   (node box vs your box)
    resolution  prunes in DEPTH   (node spacing vs your requested spacing)

--------------------------------------------------------------------------
ANACONDA

    conda env create -f python/environment.yml
    conda activate copc

or into an env you already have:

    conda install -c conda-forge laspy lazrs numpy vtk
    # optional, only for --backend pdal:
    conda install -c conda-forge python-pdal

--------------------------------------------------------------------------
EXAMPLES

    # what is in this file? (header only, no points)
    python python/copc_partial_load_vtk.py lone-star.copc.laz --info

    # load the centre 25 % of the extent, coarse, and render it
    python python/copc_partial_load_vtk.py lone-star.copc.laz --resolution 0.2

    # an explicit box at full detail
    python python/copc_partial_load_vtk.py lone-star.copc.laz \
        --bounds 515370,515390,4918350,4918370

    # straight off S3 -- only the needed byte ranges are fetched
    python python/copc_partial_load_vtk.py \
        https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz \
        --bounds 636000,636500,850000,850500 --resolution 2

    # the same box at five detail levels, no window
    python python/copc_partial_load_vtk.py lone-star.copc.laz --ladder

    # headless
    python python/copc_partial_load_vtk.py lone-star.copc.laz \
        --offscreen --screenshot out.png
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# The inspector lives next to this file and does the index-only work: header
# peek, octree parse, and costing a query before it runs. Reusing it keeps one
# implementation of the COPC parsing.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from copc_hierarchy_inspect import (  # noqa: E402
    CopcInfo,
    LasHeader,
    cost_query,
    open_source,
    parse_copc_info,
    parse_las_header,
    read_hierarchy,
)


# ============================================================================
# PART 1 -- peek at the file without reading any points
#
# Extent and point count come out of the 375-byte LAS header, and the octree
# structure out of the hierarchy pages. Together that is usually a couple of KB,
# and it is everything you need to decide WHAT to ask for.
# ============================================================================


@dataclass
class Peeked:
    header: LasHeader
    info: CopcInfo
    nodes: list
    size: int
    index_bytes: int
    index_requests: int


def peek(path_or_url: str) -> Peeked:
    source = open_source(path_or_url)

    prefix = source.read_range(0, 589)

    header = parse_las_header(prefix)
    info = parse_copc_info(prefix)

    nodes, _pages = read_hierarchy(source, info)

    peeked = Peeked(
        header=header,
        info=info,
        nodes=nodes,
        size=source.size,
        index_bytes=source.bytes_read,
        index_requests=source.requests,
    )

    if hasattr(source, "close"):
        source.close()

    return peeked


# ============================================================================
# PART 2 -- the partial load
#
# laspy's CopcReader does the octree walk. It takes the same two arguments the
# C++ example hands to PDAL, and it accepts a URL directly -- fetching byte
# ranges over HTTP exactly as a browser-based viewer would.
# ============================================================================


@dataclass
class Loaded:
    xyz: "object"          # numpy (N, 3) float64
    count: int
    seconds: float


def load_laspy(path_or_url: str,
               box: tuple[float, float, float, float],
               resolution: float | None,
               zmin: float,
               zmax: float) -> Loaded:
    import numpy as np
    from laspy.copc import Bounds, CopcReader

    started = time.perf_counter()

    query_bounds = Bounds(
        mins=np.asarray([box[0], box[2], zmin]),
        maxs=np.asarray([box[1], box[3], zmax]),
    )

    if path_or_url.startswith(("http://", "https://")):
        opener = CopcReader.open(path_or_url)
    else:
        opener = CopcReader.open(open(path_or_url, "rb"))

    with opener as reader:
        # bounds -> prune in space;  resolution -> prune in depth.
        points = reader.query(bounds=query_bounds, resolution=resolution)

    xyz = (
        np.stack([points.x, points.y, points.z], axis=1)
        if len(points)
        else np.empty((0, 3))
    )

    return Loaded(xyz=xyz, count=len(xyz),
                  seconds=time.perf_counter() - started)


def load_pdal(path: str,
              box: tuple[float, float, float, float],
              resolution: float | None) -> Loaded:
    """
    The same query through PDAL, for parity with the C++ examples.

    PDAL takes the bounds as a string in its own spelling and is happy to run
    the whole thing as a JSON pipeline.
    """
    import numpy as np
    import pdal

    started = time.perf_counter()

    stage = {
        "type": "readers.copc",
        "filename": path,
        "bounds": f"([{box[0]},{box[1]}],[{box[2]},{box[3]}])",
    }

    if resolution:
        stage["resolution"] = resolution

    pipeline = pdal.Pipeline(json=__import__("json").dumps({"pipeline": [stage]}))

    pipeline.execute()

    arrays = pipeline.arrays

    if not arrays or len(arrays[0]) == 0:
        return Loaded(xyz=np.empty((0, 3)), count=0,
                      seconds=time.perf_counter() - started)

    array = arrays[0]

    xyz = np.stack([array["X"], array["Y"], array["Z"]], axis=1)

    return Loaded(xyz=xyz, count=len(xyz),
                  seconds=time.perf_counter() - started)


# ============================================================================
# PART 3 -- numpy -> VTK
#
# vtkPolyData needs two things for points to be drawn: the coordinates, and one
# vertex CELL per point. A polydata with points but no cells renders an empty
# window, which is the single most common way this goes wrong.
#
# numpy_to_vtk gives a zero-copy view of the coordinate array, so the points
# are never duplicated in memory. The `deep=True` is deliberate: the numpy
# array is local to this function and would otherwise be freed under VTK.
# ============================================================================


def to_polydata(xyz):
    import numpy as np
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

    count = len(xyz)

    polydata = vtk.vtkPolyData()

    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.ascontiguousarray(xyz, dtype=np.float64),
                                deep=True))
    polydata.SetPoints(points)

    # A vtkCellArray of verts wants [1, i0, 1, i1, 1, i2, ...].
    connectivity = np.empty((count, 2), dtype=np.int64)
    connectivity[:, 0] = 1
    connectivity[:, 1] = np.arange(count, dtype=np.int64)

    cells = vtk.vtkCellArray()
    cells.SetCells(count,
                   numpy_to_vtkIdTypeArray(connectivity.ravel(), deep=True))
    polydata.SetVerts(cells)

    # Colour by elevation.
    elevation = numpy_to_vtk(np.ascontiguousarray(xyz[:, 2], dtype=np.float64),
                             deep=True)
    elevation.SetName("Elevation")
    polydata.GetPointData().SetScalars(elevation)

    return polydata


def render(polydata,
           box: tuple[float, float, float, float],
           zmin: float,
           zmax: float,
           full_extent: tuple[float, float, float, float],
           title: str,
           offscreen: bool,
           screenshot: str | None,
           point_size: float) -> None:
    import vtk

    lut = vtk.vtkLookupTable()
    lut.SetHueRange(0.667, 0.0)          # blue (low) -> red (high)
    lut.SetTableRange(zmin, zmax)
    lut.Build()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    mapper.SetLookupTable(lut)
    mapper.SetScalarRange(zmin, zmax)
    mapper.ScalarVisibilityOn()

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetPointSize(point_size)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.08, 0.10, 0.15)
    renderer.AddActor(actor)

    # The box we asked for, and the whole dataset, so the ratio is visible.
    for bounds, colour, width in (
        ((box[0], box[1], box[2], box[3]), (1.0, 0.6, 0.2), 2.0),
        (full_extent, (0.35, 0.40, 0.55), 1.0),
    ):
        cube = vtk.vtkCubeSource()
        cube.SetBounds(bounds[0], bounds[1], bounds[2], bounds[3], zmin, zmax)

        cube_mapper = vtk.vtkPolyDataMapper()
        cube_mapper.SetInputConnection(cube.GetOutputPort())

        cube_actor = vtk.vtkActor()
        cube_actor.SetMapper(cube_mapper)
        cube_actor.GetProperty().SetRepresentationToWireframe()
        cube_actor.GetProperty().SetColor(*colour)
        cube_actor.GetProperty().SetLineWidth(width)
        cube_actor.PickableOff()

        renderer.AddActor(cube_actor)

    scalar_bar = vtk.vtkScalarBarActor()
    scalar_bar.SetLookupTable(lut)
    scalar_bar.SetTitle("Z")
    scalar_bar.SetNumberOfLabels(5)
    # Without these, vtkScalarBarActor scales its title to the bar's bounding
    # box and the "Z" ends up the size of the window.
    scalar_bar.SetWidth(0.07)
    scalar_bar.SetHeight(0.45)
    scalar_bar.SetPosition(0.90, 0.10)
    scalar_bar.UnconstrainedFontSizeOn()

    for text_property in (scalar_bar.GetTitleTextProperty(),
                          scalar_bar.GetLabelTextProperty()):
        text_property.SetFontSize(14)
        text_property.ItalicOff()
        text_property.BoldOff()
        text_property.ShadowOff()

    caption = vtk.vtkTextActor()
    caption.SetInput(title)
    caption.GetTextProperty().SetFontSize(16)
    caption.GetTextProperty().SetColor(0.9, 0.9, 0.6)
    caption.SetPosition(12, 12)

    # AddViewProp rather than AddActor2D: the C++ vtkViewport::AddActor2D is
    # not exposed on vtkRenderer in the Python bindings, and AddViewProp takes
    # 2D and 3D props alike.
    renderer.AddViewProp(scalar_bar)
    renderer.AddViewProp(caption)

    # Frame the LOADED points, not every actor. The full-extent wireframe is
    # deliberately allowed to run off screen -- it is there for scale, and
    # letting it drive the camera would shrink a small query to a few pixels.
    bounds = polydata.GetBounds()

    if polydata.GetNumberOfPoints():
        renderer.ResetCamera(bounds[0], bounds[1], bounds[2], bounds[3],
                             bounds[4], bounds[5])
    else:
        renderer.ResetCamera()

    window = vtk.vtkRenderWindow()
    window.AddRenderer(renderer)
    window.SetSize(1200, 800)
    window.SetWindowName("COPC partial load")

    if offscreen:
        window.SetOffScreenRendering(1)

    window.Render()

    if screenshot:
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.Update()

        writer = vtk.vtkPNGWriter()
        writer.SetFileName(screenshot)
        writer.SetInputConnection(capture.GetOutputPort())
        writer.Write()

        print(f"\nWrote {screenshot}")

    if not offscreen:
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(window)
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()
        interactor.Start()


# ============================================================================
# MAIN
# ============================================================================


def parse_box(text: str) -> tuple[float, float, float, float]:
    parts = [float(v) for v in text.split(",")]

    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected xmin,xmax,ymin,ymax")

    return parts[0], parts[1], parts[2], parts[3]


def describe(peeked: Peeked, source_name: str) -> None:
    header = peeked.header

    print("\n=== FILE ====================================================")
    print(f"Source        : {source_name}")
    print(f"Total points  : {header.point_count:,}")
    print(f"File size     : {peeked.size:,} bytes "
          f"({peeked.size / 1024 / 1024:.2f} MB)")
    print(f"Extent X      : {header.minx:.3f} ... {header.maxx:.3f}  "
          f"({header.maxx - header.minx:.3f})")
    print(f"Extent Y      : {header.miny:.3f} ... {header.maxy:.3f}  "
          f"({header.maxy - header.miny:.3f})")
    print(f"Extent Z      : {header.minz:.3f} ... {header.maxz:.3f}  "
          f"({header.maxz - header.minz:.3f})")
    print(f"Octree        : {len(peeked.nodes)} nodes, "
          f"{max((n.key.level for n in peeked.nodes), default=0) + 1} levels, "
          f"root spacing {peeked.info.spacing:.4f}")
    print(f"\n-> Learned from {peeked.index_bytes:,} bytes in "
          f"{peeked.index_requests} request(s). No points read yet.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Load one box of a COPC cloud at one level of detail and "
                    "render it in VTK. Works on a local file or a URL.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("EXAMPLES")[-1],
    )

    parser.add_argument("source", help="path to a .copc.laz file, or an http(s) URL")
    parser.add_argument("--info", action="store_true",
                        help="print header/octree info and exit")
    parser.add_argument("--bounds", type=parse_box, metavar="xmin,xmax,ymin,ymax",
                        help="region to load")
    parser.add_argument("--fraction", type=float, default=0.25, metavar="F",
                        help="instead of --bounds, load the centre F (0..1) "
                             "of the extent (default 0.25)")
    parser.add_argument("--resolution", type=float, metavar="R",
                        help="target point spacing (omit for full detail)")
    parser.add_argument("--ladder", action="store_true",
                        help="load the same box at five detail levels, no render")
    parser.add_argument("--backend", choices=("laspy", "pdal"), default="laspy",
                        help="which library performs the query (default laspy)")
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--offscreen", action="store_true",
                        help="render without opening a window")
    parser.add_argument("--screenshot", metavar="PNG",
                        help="write the rendered view to a PNG")

    args = parser.parse_args()

    try:
        peeked = peek(args.source)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    header = peeked.header

    describe(peeked, args.source)

    if args.info:
        return 0

    # ------------------------------------------------------------------------
    # Decide what to ask for.
    # ------------------------------------------------------------------------

    if args.bounds:
        box = args.bounds
    else:
        cx = 0.5 * (header.minx + header.maxx)
        cy = 0.5 * (header.miny + header.maxy)
        hx = 0.5 * args.fraction * (header.maxx - header.minx)
        hy = 0.5 * args.fraction * (header.maxy - header.miny)

        box = (cx - hx, cx + hx, cy - hy, cy + hy)

        print(f"\nNo --bounds given, using the centre "
              f"{args.fraction * 100:g} % of the extent.")

    full_extent = (header.minx, header.maxx, header.miny, header.maxy)

    area_share = (
        100.0 * (box[1] - box[0]) * (box[3] - box[2])
        / max((header.maxx - header.minx) * (header.maxy - header.miny), 1e-9)
    )

    if args.backend == "pdal" and args.source.startswith(("http://", "https://")):
        print("\nnote: --backend pdal needs a local file; use laspy for URLs.",
              file=sys.stderr)
        return 1

    def run(resolution: float | None) -> Loaded:
        if args.backend == "pdal":
            return load_pdal(args.source, box, resolution)

        return load_laspy(args.source, box, resolution,
                          header.minz, header.maxz)

    # ------------------------------------------------------------------------
    # --ladder: identical bounds, five resolutions. Only one thing changes.
    # ------------------------------------------------------------------------

    if args.ladder:
        print("\n=== LOD LADDER ==============================================")
        print(f"bounds fixed at ([{box[0]:.3f},{box[1]:.3f}],"
              f"[{box[2]:.3f},{box[3]:.3f}]) -- {area_share:.2f} % of the extent.")
        print("Only `resolution` changes between rows.\n")

        print(" resolution       points    % of file      MB     seconds")
        print(" ----------   ----------   ----------   -----   ---------")

        max_level = max((n.key.level for n in peeked.nodes), default=0)

        for level in range(max_level + 1):
            resolution = peeked.info.spacing_at(level)
            loaded = run(resolution)

            print(f" {resolution:10.4f}   {loaded.count:10,}   "
                  f"{100.0 * loaded.count / max(header.point_count, 1):9.2f} %   "
                  f"{loaded.xyz.nbytes / 1024 / 1024:5.1f}   "
                  f"{loaded.seconds:9.3f}")

        loaded = run(None)

        print(f" {'(none)':>10}   {loaded.count:10,}   "
              f"{100.0 * loaded.count / max(header.point_count, 1):9.2f} %   "
              f"{loaded.xyz.nbytes / 1024 / 1024:5.1f}   {loaded.seconds:9.3f}")

        print("\nThe file, the bounds and the code were identical in every row.")

        return 0

    # ------------------------------------------------------------------------
    # The single partial load, with its cost predicted first.
    # ------------------------------------------------------------------------

    print("\n=== QUERY ===================================================")
    print(f"bounds     : ([{box[0]:.3f},{box[1]:.3f}],[{box[2]:.3f},{box[3]:.3f}])")
    print(f"             {box[1] - box[0]:.3f} x {box[3] - box[2]:.3f} units "
          f"({area_share:.2f} % of the extent)")
    print(f"resolution : "
          f"{args.resolution if args.resolution else '(none -- full detail)'}")

    # Predicted from the index alone, before any point data moves.
    cost = cost_query(peeked.nodes, peeked.info, box, args.resolution)

    print(f"\nPredicted from the index:")
    print(f"  nodes to read    : {cost.nodes} of {len(peeked.nodes)}"
          f"   ({cost.culled_by_bounds} culled by bounds, "
          f"{cost.culled_by_depth} by depth)")

    if cost.max_level is not None:
        print(f"  deepest level    : {cost.max_level} "
              f"(spacing {peeked.info.spacing_at(cost.max_level):.4f})")

    print(f"  bytes to read    : {cost.data_bytes:,} of {peeked.size:,}"
          f"   ({100.0 * cost.data_bytes / max(peeked.size, 1):.2f} %)")
    print(f"  points in those nodes: {cost.points:,}  (upper bound -- the "
          f"reader then crops to the box)")

    loaded = run(args.resolution)

    print(f"\nActual ({args.backend}):")
    print(f"  points loaded    : {loaded.count:,} of {header.point_count:,}"
          f"   ({100.0 * loaded.count / max(header.point_count, 1):.2f} %)")
    print(f"  coordinate array : {loaded.xyz.nbytes / 1024 / 1024:.2f} MB")
    print(f"  load time        : {loaded.seconds:.3f} s")

    if cost.points:
        print(f"  kept / read      : "
              f"{100.0 * loaded.count / cost.points:.1f} % of the points that "
              f"had to be decompressed")
        print("     (a node is the smallest unit of I/O, so whole nodes are")
        print("      read and then cropped -- ask for boxes comparable to the")
        print("      node size and this ratio goes up)")

    print(f"\n-> {loaded.count:,} points are in memory. The other "
          f"{header.point_count - loaded.count:,} were never allocated"
          + (", and never downloaded."
             if args.source.startswith(("http://", "https://")) else "."))

    if loaded.count == 0:
        print("\nNothing came back -- are those bounds inside the extent?",
              file=sys.stderr)
        return 1

    # ------------------------------------------------------------------------
    # Render.
    # ------------------------------------------------------------------------

    try:
        polydata = to_polydata(loaded.xyz)
    except ImportError:
        print("\nVTK is not installed, so there is nothing to render.")
        print("    conda install -c conda-forge vtk")
        return 0

    title = (f"{loaded.count:,} points  |  "
             f"{100.0 * loaded.count / max(header.point_count, 1):.2f} % of file  |  "
             f"resolution "
             f"{args.resolution if args.resolution else 'full'}")

    render(polydata, box, header.minz, header.maxz, full_extent, title,
           args.offscreen, args.screenshot, args.point_size)

    return 0


if __name__ == "__main__":
    sys.exit(main())
