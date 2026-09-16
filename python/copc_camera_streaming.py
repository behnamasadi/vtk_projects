#!/usr/bin/env python3
"""
copc_camera_streaming.py -- let the camera decide what to load.

The Python twin of src/copc_camera_streaming.cpp, and the example where
nothing about the level of detail is hard-coded. Every time the camera stops
moving, BOTH query options are re-derived from it:

    camera position + focal point + view angle + window size
            |
            +--> frustum planes ........ -> visible XY box   (WHERE)
            +--> screen-space error ..... -> resolution       (HOW DETAILED)
                        |
                        v
            laspy CopcReader.query(bounds=..., resolution=...)
                        |
                        v
            a new vtkPolyData replaces the old one

Nobody ever names an octree level. The level is whatever the traversal reaches
before the node spacing gets finer than the number that falls out of the
screen-space-error division.

--------------------------------------------------------------------------
ANACONDA

    conda env create -f python/environment.yml
    conda activate copc

or:  conda install -c conda-forge laspy lazrs numpy vtk

--------------------------------------------------------------------------
EXAMPLES

    # interactive; works on a local file or straight off S3
    python python/copc_camera_streaming.py lone-star.copc.laz

    python python/copc_camera_streaming.py \
        https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz \
        --budget 400000 --error 2.5

    # headless: fly the camera through a scripted zoom and write a PNG per
    # step, printing every decision. This is how the loop gets tested without
    # a human holding the mouse.
    python python/copc_camera_streaming.py lone-star.copc.laz \
        --demo 5 --screenshot-prefix step

Keys (interactive):
    r   force a reload now
    b   toggle the wireframe box showing the current query region
    q   quit
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from copc_hierarchy_inspect import cost_query  # noqa: E402
from copc_partial_load_vtk import load_laspy, peek, to_polydata  # noqa: E402


# ============================================================================
# The streaming viewer.
# ============================================================================


class CopcStreamer:
    def __init__(self, source: str, peeked, *,
                 target_pixel_error: float = 2.0,
                 point_budget: int = 500_000) -> None:
        self.source = source
        self.peeked = peeked
        self.header = peeked.header

        self.full_extent = (self.header.minx, self.header.maxx,
                            self.header.miny, self.header.maxy)

        # POLICY. These two numbers are the only dials in the program.
        #
        # target_pixel_error: how far apart, in PIXELS, we are willing to let
        # neighbouring points sit on screen. 1-3 px looks solid. It is in
        # pixels on purpose -- that is a unit a human can reason about,
        # unlike "level 4".
        self.target_pixel_error = target_pixel_error

        # point_budget: the hard ceiling. Frame rate is the real constraint,
        # not fidelity.
        self.point_budget = point_budget

        self.renderer = None
        self.cloud_actor = None
        self.box_actor = None
        self.hud = None

        self.last_box = None
        self.last_resolution = None
        self.loaded_count = 0

    # ------------------------------------------------------------------
    # WHERE: frustum -> XY box.
    #
    # vtkCamera.GetFrustumPlanes gives six inward-facing planes. Feeding them
    # to vtkPlanes + vtkFrustumSource builds the actual frustum geometry,
    # whose axis-aligned bounds are a conservative superset of what is
    # visible. Conservative is the right error: ask for slightly too much and
    # nothing pops in at the edge of the screen while panning.
    # ------------------------------------------------------------------
    def visible_box(self):
        import vtk

        camera = self.renderer.GetActiveCamera()

        width, height = self.renderer.GetRenderWindow().GetSize()
        aspect = max(width, 1) / max(height, 1)

        planes = [0.0] * 24
        camera.GetFrustumPlanes(aspect, planes)

        frustum_planes = vtk.vtkPlanes()
        frustum_planes.SetFrustumPlanes(planes)

        frustum = vtk.vtkFrustumSource()
        frustum.SetPlanes(frustum_planes)
        frustum.ShowLinesOff()
        frustum.Update()

        b = frustum.GetOutput().GetBounds()

        # Clamp to the dataset; never ask for empty space.
        box = (
            max(b[0], self.full_extent[0]),
            min(b[1], self.full_extent[1]),
            max(b[2], self.full_extent[2]),
            min(b[3], self.full_extent[3]),
        )

        if box[1] <= box[0] or box[3] <= box[2]:
            return None

        return box

    # ------------------------------------------------------------------
    # HOW DETAILED: screen-space error -> a point spacing in world units.
    #
    #     focal_length_pixels = height / (2 * tan(fov / 2))
    #
    # A world length L at distance D covers  L / D * focal_length_pixels
    # pixels. Invert it: to make neighbouring points land target_pixel_error
    # apart on screen,
    #
    #     resolution = target_pixel_error * D / focal_length_pixels
    #
    # Move closer, D shrinks, resolution shrinks, the octree walk descends
    # deeper. That is the entire LOD decision.
    # ------------------------------------------------------------------
    def resolution_for_camera(self):
        camera = self.renderer.GetActiveCamera()

        position = camera.GetPosition()
        focal = camera.GetFocalPoint()

        distance = math.dist(position, focal)

        _width, height = self.renderer.GetRenderWindow().GetSize()
        height = max(height, 1)

        if camera.GetParallelProjection():
            # Orthographic: on-screen size does not depend on distance at all.
            # ParallelScale is half the visible height in world units.
            world_per_pixel = (2.0 * camera.GetParallelScale()) / height

            return self.target_pixel_error * world_per_pixel, distance, None

        fov = math.radians(camera.GetViewAngle())
        focal_length_pixels = height / (2.0 * math.tan(fov / 2.0))

        if focal_length_pixels < 1e-9:
            return None, distance, focal_length_pixels

        resolution = self.target_pixel_error * distance / focal_length_pixels

        return resolution, distance, focal_length_pixels

    # ------------------------------------------------------------------
    # "Close enough that reloading would not change the picture."
    # ------------------------------------------------------------------
    def is_similar(self, box, resolution) -> bool:
        if self.last_box is None or not self.last_resolution:
            return False

        ratio = resolution / self.last_resolution

        # A <25 % change in requested spacing rarely even changes the depth.
        if ratio < 0.8 or ratio > 1.25:
            return False

        tol_x = 0.2 * (self.last_box[1] - self.last_box[0])
        tol_y = 0.2 * (self.last_box[3] - self.last_box[2])

        return (abs(box[0] - self.last_box[0]) < tol_x
                and abs(box[1] - self.last_box[1]) < tol_x
                and abs(box[2] - self.last_box[2]) < tol_y
                and abs(box[3] - self.last_box[3]) < tol_y)

    # ------------------------------------------------------------------
    # One full decision cycle.
    # ------------------------------------------------------------------
    def update(self, force: bool = False) -> bool:
        box = self.visible_box()

        if box is None:
            print("\n[camera is looking away from the data -- nothing to load]")
            return False

        resolution, distance, focal_length_pixels = self.resolution_for_camera()

        if not force and self.is_similar(box, resolution):
            return False

        print("\n" + "=" * 60)
        print("CAMERA")
        print(f"  distance to focal point : {distance:.3f}")

        if focal_length_pixels:
            print(f"  focal length            : {focal_length_pixels:.3f} px")
        else:
            print("  projection              : parallel")

        extent_area = max(
            (self.full_extent[1] - self.full_extent[0])
            * (self.full_extent[3] - self.full_extent[2]), 1e-9)

        box_area = (box[1] - box[0]) * (box[3] - box[2])

        print("\nDERIVED QUERY")
        print(f"  visible box   : ([{box[0]:.3f},{box[1]:.3f}],"
              f"[{box[2]:.3f},{box[3]:.3f}])")
        print(f"                  {box[1] - box[0]:.1f} x {box[3] - box[2]:.1f} "
              f"units ({100.0 * box_area / extent_area:.2f} % of extent)")

        if focal_length_pixels:
            print(f"  resolution    : {resolution:.5f}  "
                  f"(= {self.target_pixel_error} px * {distance:.3f} / "
                  f"{focal_length_pixels:.3f})")
        else:
            print(f"  resolution    : {resolution:.5f}")

        # ------------------------------------------------------------------
        # BUDGET, enforced from the index -- before any point data moves.
        #
        # We hold the octree index in memory, so we can COST every candidate
        # level against this box and pick the deepest one that fits, in a
        # single pass of arithmetic. cost_query returns the points inside the
        # nodes that would be read, which is an UPPER BOUND on what the reader
        # hands back after cropping -- so a level that fits the budget by that
        # measure is guaranteed to fit for real.
        #
        # The naive alternative -- query, notice it is too big, double the
        # resolution, query again -- costs a full network round trip per
        # guess, and can waste one entirely when doubling the resolution does
        # not happen to change the depth. Having the index client-side is what
        # buys us this, and it is the main thing a real viewer does that a
        # one-shot PDAL call cannot.
        # ------------------------------------------------------------------
        wanted_level = cost_query(self.peeked.nodes, self.peeked.info, box,
                                  resolution).max_level

        chosen_level = 0
        cost = cost_query(self.peeked.nodes, self.peeked.info, box,
                          self.peeked.info.spacing_at(0))

        for level in range(wanted_level, -1, -1):
            candidate = cost_query(self.peeked.nodes, self.peeked.info, box,
                                   self.peeked.info.spacing_at(level))

            if candidate.points <= self.point_budget:
                chosen_level, cost = level, candidate
                break

        coarsened = wanted_level - chosen_level

        if coarsened:
            resolution = self.peeked.info.spacing_at(chosen_level)

        print(f"  index says    : level {wanted_level} would read "
              f"{cost_query(self.peeked.nodes, self.peeked.info, box, self.peeked.info.spacing_at(wanted_level)).data_bytes:,}"
              f" bytes")
        print(f"  budget picks  : level {chosen_level} -> {cost.nodes} of "
              f"{len(self.peeked.nodes)} nodes, {cost.data_bytes:,} bytes, "
              f"<= {cost.points:,} points")

        if coarsened:
            print(f"  (coarsened {coarsened} level(s) to stay under "
                  f"{self.point_budget:,}; resolution now {resolution:.5f})")

        # Exactly ONE query, at a level we already know fits.
        loaded = load_laspy(self.source, box, resolution,
                            self.header.minz, self.header.maxz)

        print("\nRESULT")
        print(f"  points loaded : {loaded.count:,}  "
              f"({100.0 * loaded.count / max(self.header.point_count, 1):.2f} % "
              f"of the file)")
        print(f"  load time     : {loaded.seconds:.3f} s")

        print(f"  queries made  : 1  (the level was chosen from the index)")

        # Swap the geometry under the existing actor: no actor churn, and the
        # camera is never reset, so the view stays where the user put it.
        self.cloud_actor.GetMapper().SetInputData(to_polydata(loaded.xyz))

        self.update_box(box)
        self.update_hud(box, resolution, loaded.count, distance)

        self.last_box = box
        self.last_resolution = resolution
        self.loaded_count = loaded.count

        self.renderer.GetRenderWindow().Render()

        return True

    def update_box(self, box) -> None:
        if self.box_actor is None:
            return

        cube = self.box_actor.GetMapper().GetInputAlgorithm()

        cube.SetBounds(box[0], box[1], box[2], box[3],
                       self.header.minz, self.header.maxz)

    def update_hud(self, box, resolution, count, distance) -> None:
        if self.hud is None:
            return

        self.hud.SetInput(
            f"distance   {distance:.2f}\n"
            f"resolution {resolution:.4f}\n"
            f"box        {box[1] - box[0]:.1f} x {box[3] - box[2]:.1f}\n"
            f"points     {count:,} / {self.header.point_count:,}\n"
            f"budget     {self.point_budget:,}   error "
            f"{self.target_pixel_error} px"
        )


# ============================================================================
# Scene
# ============================================================================


def build_scene(streamer: CopcStreamer, offscreen: bool):
    import vtk

    header = streamer.header

    lut = vtk.vtkLookupTable()
    lut.SetHueRange(0.667, 0.0)
    lut.SetTableRange(header.minz, header.maxz)
    lut.Build()

    cloud_mapper = vtk.vtkPolyDataMapper()
    cloud_mapper.SetInputData(vtk.vtkPolyData())      # starts empty
    cloud_mapper.SetLookupTable(lut)
    cloud_mapper.SetScalarRange(header.minz, header.maxz)
    cloud_mapper.ScalarVisibilityOn()

    cloud_actor = vtk.vtkActor()
    cloud_actor.SetMapper(cloud_mapper)
    cloud_actor.GetProperty().SetPointSize(2.0)

    cube = vtk.vtkCubeSource()
    cube.SetBounds(header.minx, header.maxx, header.miny, header.maxy,
                   header.minz, header.maxz)

    box_mapper = vtk.vtkPolyDataMapper()
    box_mapper.SetInputConnection(cube.GetOutputPort())

    box_actor = vtk.vtkActor()
    box_actor.SetMapper(box_mapper)
    box_actor.GetProperty().SetRepresentationToWireframe()
    box_actor.GetProperty().SetColor(1.0, 0.6, 0.2)
    box_actor.GetProperty().SetLineWidth(2.0)
    box_actor.PickableOff()

    hud = vtk.vtkTextActor()
    hud.SetInput("loading...")
    hud.GetTextProperty().SetFontSize(16)
    hud.GetTextProperty().SetColor(0.9, 0.9, 0.6)
    hud.SetPosition(12, 12)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.08, 0.10, 0.15)
    renderer.AddActor(cloud_actor)
    renderer.AddActor(box_actor)
    renderer.AddViewProp(hud)          # AddActor2D is not wrapped in Python

    window = vtk.vtkRenderWindow()
    window.AddRenderer(renderer)
    window.SetSize(1200, 800)
    window.SetWindowName("COPC camera-driven LOD streaming")

    if offscreen:
        window.SetOffScreenRendering(1)

    streamer.renderer = renderer
    streamer.cloud_actor = cloud_actor
    streamer.box_actor = box_actor
    streamer.hud = hud

    # Frame the whole dataset, so the first query is a coarse overview.
    renderer.ResetCamera(header.minx, header.maxx, header.miny, header.maxy,
                         header.minz, header.maxz)

    window.Render()

    return window


def screenshot(window, path: str) -> None:
    import vtk

    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.Modified()
    capture.Update()

    writer = vtk.vtkPNGWriter()
    writer.SetFileName(path)
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()

    print(f"  wrote {path}")


# ============================================================================
# MAIN
# ============================================================================


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Camera-driven COPC streaming: the frustum picks the "
                    "bounds and the screen-space error picks the resolution.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("EXAMPLES")[-1],
    )

    parser.add_argument("source", help="path to a .copc.laz file, or an http(s) URL")
    parser.add_argument("--budget", type=int, default=500_000,
                        help="max points to hold at once (default 500000)")
    parser.add_argument("--error", type=float, default=2.0, metavar="PIXELS",
                        help="target on-screen point spacing (default 2.0)")
    parser.add_argument("--demo", type=int, metavar="N",
                        help="headless: fly a scripted zoom of N steps, "
                             "printing every decision")
    parser.add_argument("--screenshot-prefix", metavar="PREFIX",
                        help="with --demo, write PREFIX_00.png per step")

    args = parser.parse_args()

    try:
        peeked = peek(args.source)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    header = peeked.header

    streamer = CopcStreamer(args.source, peeked,
                            target_pixel_error=args.error,
                            point_budget=args.budget)

    print(f"\nFile          : {args.source}")
    print(f"Total points  : {header.point_count:,}")
    print(f"Extent        : ([{header.minx:.3f},{header.maxx:.3f}],"
          f"[{header.miny:.3f},{header.maxy:.3f}])")
    print(f"Octree        : {len(peeked.nodes)} nodes, root spacing "
          f"{peeked.info.spacing:.4f}")
    print(f"Point budget  : {args.budget:,}")
    print(f"Target error  : {args.error} px")

    try:
        import vtk
    except ImportError:
        print("\nVTK is not installed.\n    conda install -c conda-forge vtk",
              file=sys.stderr)
        return 1

    window = build_scene(streamer, offscreen=bool(args.demo))

    # ------------------------------------------------------------------
    # Headless demo: drive the camera ourselves so the loop can be checked
    # without a human at the mouse. Each step dollies in by 1.8x, which is
    # roughly one octree level per step.
    # ------------------------------------------------------------------
    if args.demo:
        camera = streamer.renderer.GetActiveCamera()

        print(f"\nScripted zoom, {args.demo} steps, 1.8x closer each time.\n")

        for step in range(args.demo):
            if step:
                camera.Dolly(1.8)
                streamer.renderer.ResetCameraClippingRange()

            print(f"\n----- step {step} " + "-" * 44)

            streamer.update(force=True)

            if args.screenshot_prefix:
                screenshot(window, f"{args.screenshot_prefix}_{step:02d}.png")

        print("\nNote how the resolution number falls while the point count")
        print("stays in the same range. That flatness is what LOD buys: cost")
        print("roughly independent of how close you get, or how big the file is.")

        return 0

    # ------------------------------------------------------------------
    # Interactive.
    # ------------------------------------------------------------------
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(window)

    style = vtk.vtkInteractorStyleTrackballCamera()
    interactor.SetInteractorStyle(style)

    print("\nDrag / zoom / pan. Every mouse release re-derives the query.")
    print("Keys: r = reload, b = toggle query box, q = quit")

    # First load, before any interaction.
    streamer.update(force=True)

    # EndInteractionEvent fires once, on mouse release. That is the right
    # moment for a synchronous reload: the drag itself never blocks.
    def on_end_interaction(_caller, _event):
        streamer.update(force=False)

    style.AddObserver(vtk.vtkCommand.EndInteractionEvent, on_end_interaction)

    def on_key(caller, _event):
        key = caller.GetKeySym()

        if key == "r":
            print("\n[forced reload]")
            streamer.update(force=True)
        elif key == "b":
            streamer.box_actor.SetVisibility(
                not streamer.box_actor.GetVisibility())
            window.Render()

    interactor.AddObserver(vtk.vtkCommand.KeyPressEvent, on_key)

    interactor.Initialize()
    interactor.Start()

    return 0


if __name__ == "__main__":
    sys.exit(main())
