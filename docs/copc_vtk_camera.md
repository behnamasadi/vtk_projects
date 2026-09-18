# The VTK camera and the streaming loop

> [index](copc_laz_lod_tutorial.md) · [1. Format](copc_format.md) · [2. LOD mechanisms](copc_lod_mechanisms.md) · [3. PDAL](copc_pdal.md) · **4. VTK camera** · [5. Worked examples](copc_worked_examples.md)

Where the LOD decision comes from: position, focal point, frustum planes,
screen-space error — and the loop that turns them into one COPC query.

---

## 6. The VTK camera — where the LOD decision comes from

[src/camera_lod_COCP.cpp](../src/camera_lod_COCP.cpp) is the other half. It has
no PDAL in it: it isolates the viewer-side question, *given where the camera is,
how much detail do I need?*

### Position, focal point, view up

```
                         view up
                            ▲
                            │
    camera position ●───────┼──────────────────▶ ● focal point
       (the eye)      direction of projection      (the aim point
                            │                       AND the pivot)
                       ┌────┴────┐
                       │ frustum │
```

* **Position** — the eye, in world coordinates.
* **Focal point** — what the camera looks at. `direction = focal − position`.
* **View up** — roll; which way is up in the image.
* **View angle** — vertical field of view in degrees (default 30°).
* **Clipping range** — near and far distances along the view direction.
* **Distance** = `|focal − position|`, the single number that drives LOD.

`vtkInteractorStyleTrackballCamera` **rotates the camera around the focal
point**. Not around the origin, not around the object — around whatever the
focal point currently is. Pan moves position *and* focal point together; dolly
moves position toward the focal point; `Zoom` changes the view angle instead.
This is why "the object spins off screen" bugs are almost always a focal point
left somewhere stale. See also [docs/camera_position.md](camera_position.md) and
[docs/modify_renderer_camera.md](modify_renderer_camera.md).

The demo makes the pivot literal:

| Visual | Meaning |
|---|---|
| Red sphere + billboard label | current focal point = rotation pivot |
| Yellow line | camera position → focal point |
| Green wireframe | the frustum, from `GetFrustumPlanes` |
| Axes at origin | world reference, so you can see the pivot move away from it |

Drag to rotate: the sphere stays put. Middle-drag to pan: the sphere moves with
you, because the callback re-reads the focal point on every `ModifiedEvent` and
calls `SetCenter`/`SetPosition`/`SetPoint1`/`SetPoint2`.

### Frustum planes

```cpp
double planes[24];
camera->GetFrustumPlanes(aspect, planes);   // 6 planes × (A,B,C,D)
```

Six planes as `Ax + By + Cz + D = 0`, **normals pointing inward** — so a point is
inside the frustum iff all six give `Ax+By+Cz+D ≥ 0`.

The order is a classic trap, and the demo gets it right:

```
index 0  LEFT    (-x)
index 1  RIGHT   (+x)
index 2  BOTTOM  (-y)
index 3  TOP     (+y)
index 4  FAR     (-z)     ← far before near, not near before far
index 5  NEAR    (+z)
```

`aspect` must be the viewport's width/height or the left/right planes are wrong.
More in [docs/frustum.md](frustum.md) and
[docs/point_visibility_in_camera_frustum.md](point_visibility_in_camera_frustum.md).

Feed them back into geometry with `vtkPlanes::SetFrustumPlanes` +
`vtkFrustumSource` (what draws the green box), or use them to test a node's AABB
— see §7.

### Projected size: world metres → screen pixels

The one formula that connects the 3D world to the LOD level:

```
                                      height_pixels
   pixels  =   worldSize / distance  ×  ───────────────────
                                        2 · tan(fov/2)
```

```cpp
projectedPixels = (ObjectWorldSize / distance) *
                  (height / (2.0 * std::tan(viewAngleRad * 0.5)));
```

Read it as two factors: `worldSize/distance` is the angular size in radians;
`height / (2·tan(fov/2))` is the **focal length in pixels**, a constant for a
given window and FOV. Multiply and you have pixels. (Perspective only — for
`ParallelProjectionOn`, the size is `worldSize / (2·ParallelScale) × height`,
independent of distance.)

Then the policy step, `SelectCopcLOD`:

```
< 30 px  → level 0        < 240 px → level 3
< 60 px  → level 1        < 480 px → level 4
< 120 px → level 2        < 960 px → level 5
                          else     → level 6
```

Doubling thresholds because spacing halves per level — one extra level per
doubling of on-screen size. **This is viewer policy, not part of the COPC
spec.** Nothing in COPC says which level you should draw.

The honest way to say the same thing is **screen-space error**: how many pixels
apart are the points of level *d* when drawn?

```
pixelError(d) = spacing(d) / distance × focalLengthPixels
              = (rootSpacing / 2^d) / distance × focalLengthPixels

descend while pixelError(d) > targetError      (target ≈ 1–3 px)
```

Solve for *d* directly and you get the level in one step instead of a ladder of
`if`s. That formulation also generalizes per node, which is what you actually
want — a node near the camera and a node at the horizon need different depths in
the same frame.

### What the demo deliberately leaves out

Worth knowing so you are not surprised:

* There is no COPC in `camera_lod_COCP` — it prints an LOD number, it does not
  fetch anything. (The filename also spells it `COCP`.) `copc_camera_streaming`
  is the one that actually reads a file.
* `ObjectWorldSize` is a fixed 10.0 for the whole scene; a real viewer uses each
  *node's* bbox size and its own distance.
* The green frustum is built once at startup and never updated — `FrustumActor`
  is stored on the callback but unused. It shows the *initial* frustum, which is
  actually what you want for a teaching demo (fly outside it and look back), but
  it is not "the current frustum".
* The frustum planes are recomputed and printed every `ModifiedEvent`, including
  during a drag. That is a lot of `cout` — fine here, never in production.

---

## 7. Putting it together: the streaming loop

VTK gives you no streaming point-cloud reader. `vtkLODActor`, `vtkLODProp3D`
and `vtkQuadricLODActor`
([docs/actor_multiple_levels_of_detail.md](actor_multiple_levels_of_detail.md),
[docs/high_resolution_low_resolution_actor.md](high_resolution_low_resolution_actor.md))
swap between representations you have *already built in RAM* while the user
interacts. Useful, but orthogonal: they solve "don't draw 50 M points at 60 fps",
not "don't load 50 M points". The out-of-core part is yours to write.

The shape of it:

```cpp
// once
copc::Reader reader("big.copc.laz");        // or PDAL's readers.copc
std::unordered_map<VoxelKey, vtkSmartPointer<vtkActor>> loaded;

// on camera ModifiedEvent (debounced — see below)
void updateLOD()
{
    double planes[24];
    camera->GetFrustumPlanes(aspect, planes);
    double focalLenPx = height / (2.0 * std::tan(viewAngleRad * 0.5));

    std::vector<VoxelKey> wanted;
    std::queue<VoxelKey> q{{rootKey}};

    while (!q.empty())
    {
        VoxelKey k = q.front(); q.pop();
        Bounds b = boundsOf(k);                       // arithmetic, no I/O

        if (!aabbIntersectsFrustum(b, planes)) continue;   // spatial cull

        double dist = distanceTo(b, cameraPosition);
        double err  = (rootSpacing / (1 << k.level)) / dist * focalLenPx;

        wanted.push_back(k);
        if (err > targetPixelError && k.level < maxLevel)
            for (auto c : childrenOf(k))
                if (hierarchyHasNode(c)) q.push(c);        // hierarchy cull
    }

    budget(wanted, maxPointsPerFrame);         // sort by error, cut the tail

    for (auto k : wanted)
        if (!loaded.count(k))
            enqueueLoad(k);                    // worker thread: range-read,
                                               // decompress, build vtkPolyData
    for (auto& [k, actor] : loaded)
        if (!wanted.contains(k)) renderer->RemoveActor(actor);   // or keep cached
}
```

The AABB-vs-frustum test, with inward normals:

```cpp
bool aabbIntersectsFrustum(const Bounds& b, const double p[24])
{
    for (int i = 0; i < 6; ++i)
    {
        const double *n = p + i*4;
        // the box corner furthest along the inward normal
        double x = (n[0] >= 0) ? b.maxx : b.minx;
        double y = (n[1] >= 0) ? b.maxy : b.miny;
        double z = (n[2] >= 0) ? b.maxz : b.minz;
        if (n[0]*x + n[1]*y + n[2]*z + n[3] < 0)
            return false;              // entirely outside this plane
    }
    return true;
}
```

Practical rules learned the hard way:

* **One actor per node.** It makes add/remove trivial and lets VTK's own frustum
  culling ([docs/culling.md](culling.md)) do a second pass for free.
* **Never load on the camera callback thread.** A range read plus LAZ
  decompression is tens of milliseconds; doing it inline turns a smooth drag
  into a slideshow. Queue the keys, load on a worker, hand finished
  `vtkPolyData` back on the main thread, `Render()`.
* **Debounce.** `ModifiedEvent` fires many times per drag. Recompute the wanted
  set on a timer (say 100 ms) or on `EndInteractionEvent`, not on every event.
* **Cache, don't free.** Keep dropped nodes in an LRU. Rotating back and forth
  should hit RAM, not the network.
* **Budget by error.** Sort `wanted` by descending pixel error and cut at a
  point count you can draw. Frame time is what you are actually defending.
* **Prefetch coarse first.** Loading level 0 → 1 → 2 gives the user something
  immediately; loading deepest-first gives them a blank screen then a pop.
* If you go through PDAL rather than a COPC library directly, you get the same
  effect one query at a time: convert the frustum to a bbox, pick a resolution
  from the nearest point of that bbox, issue one `readers.copc` execute. Less
  precise than per-node traversal, far less code.

