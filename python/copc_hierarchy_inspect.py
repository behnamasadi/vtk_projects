#!/usr/bin/env python3
"""
copc_hierarchy_inspect.py -- read a COPC file's octree index and nothing else.

The Python twin of src/copc_hierarchy_inspect.cpp, plus the one thing that is
awkward in C++ and trivial here: it works **over HTTP**, using Range requests,
so you can inspect a multi-gigabyte COPC file on S3 while downloading about a
kilobyte.

That is not a party trick -- it IS what "cloud optimized" means. Every byte
this script reads is counted and reported, so you can see the number.

--------------------------------------------------------------------------
WHAT IT READS  (never any point data -- nothing is ever LAZ-decompressed)

    bytes 0..374      LAS 1.4 public header block   -> extent, point count
    bytes 429..588    the "copc info" VLR           -> root cube, ROOT SPACING
    the hierarchy pages it points to                -> every node's key,
                                                       byte range, point count

WHERE THE LOD COMES FROM

    A single number in the copc info VLR: `spacing`. Level d has

        spacing(d) = spacing / 2**d

    The COPC *writer* fixed that ladder when the file was made. PDAL, VTK,
    Potree and your viewer do not invent it -- they only choose which rungs
    to fetch.

--------------------------------------------------------------------------
ANACONDA

The core needs NOTHING but the standard library, so it runs in `base`:

    python python/copc_hierarchy_inspect.py file.copc.laz

The optional --load flag actually pulls points, and wants laspy + a LAZ
backend:

    conda env create -f python/environment.yml
    conda activate copc

or, into an existing env:

    conda install -c conda-forge laspy lazrs numpy

--------------------------------------------------------------------------
EXAMPLES

    # local file
    python python/copc_hierarchy_inspect.py lone-star.copc.laz

    # remote file -- reads ~1 KB of a 80 MB file
    python python/copc_hierarchy_inspect.py \
        https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz

    # what would this query cost?
    python python/copc_hierarchy_inspect.py lone-star.copc.laz \
        --bounds 515370,515380,4918340,4918350 --resolution 0.1

    # the LOD ladder, one line per resolution
    python python/copc_hierarchy_inspect.py lone-star.copc.laz --ladder

    # actually load the points (needs laspy)
    python python/copc_hierarchy_inspect.py lone-star.copc.laz \
        --bounds 515370,515380,4918340,4918350 --resolution 0.1 --load
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import urllib.request
from dataclasses import dataclass, field
from typing import Iterator

# ============================================================================
# PART 0 -- byte sources
#
# The whole argument for COPC is about WHICH BYTES you touch, so the first
# thing this script does is make byte reads explicit and countable. A local
# file and an HTTP URL differ only in how a range is fetched.
# ============================================================================


class ByteSource:
    """A random-access source of bytes that counts what it actually reads."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.bytes_read = 0
        self.requests = 0

    def read_range(self, offset: int, length: int) -> bytes:
        raise NotImplementedError

    @property
    def size(self) -> int:
        raise NotImplementedError

    def _count(self, data: bytes) -> bytes:
        self.bytes_read += len(data)
        self.requests += 1
        return data


class FileSource(ByteSource):
    def __init__(self, path: str) -> None:
        super().__init__(path)
        self._file = open(path, "rb")
        self._file.seek(0, 2)
        self._size = self._file.tell()

    @property
    def size(self) -> int:
        return self._size

    def read_range(self, offset: int, length: int) -> bytes:
        self._file.seek(offset)
        return self._count(self._file.read(length))

    def close(self) -> None:
        self._file.close()


class HttpSource(ByteSource):
    """
    Reads byte ranges with HTTP `Range:` headers.

    This needs no server software at all -- a COPC file on plain static
    hosting (S3, a CDN, nginx) is already a queryable spatial database. That
    is the entire deployment story for COPC.
    """

    def __init__(self, url: str) -> None:
        super().__init__(url)

        request = urllib.request.Request(url, method="HEAD")

        with urllib.request.urlopen(request, timeout=30) as response:
            self._size = int(response.headers.get("Content-Length", 0))
            accepts = response.headers.get("Accept-Ranges", "")

        # Not fatal -- some hosts omit the header but honour the request
        # anyway. We find out for real on the first read.
        if accepts and accepts != "bytes":
            print(
                f"warning: {url} advertises Accept-Ranges: {accepts!r}",
                file=sys.stderr,
            )

    @property
    def size(self) -> int:
        return self._size

    def read_range(self, offset: int, length: int) -> bytes:
        request = urllib.request.Request(self.name)
        request.add_header("Range", f"bytes={offset}-{offset + length - 1}")

        with urllib.request.urlopen(request, timeout=60) as response:
            if response.status != 206:
                raise RuntimeError(
                    f"server ignored the Range header (HTTP {response.status}); "
                    "it sent the whole file instead. Try a host that supports "
                    "range requests, such as S3."
                )

            return self._count(response.read())


def open_source(path_or_url: str) -> ByteSource:
    if path_or_url.startswith(("http://", "https://")):
        return HttpSource(path_or_url)

    return FileSource(path_or_url)


# ============================================================================
# PART 1 -- the LAS 1.4 public header block
#
# 375 bytes, fixed layout. Offsets straight out of the ASPRS spec.
# ============================================================================


@dataclass
class LasHeader:
    version: str
    point_format: int
    compressed: bool
    point_record_length: int
    point_count: int
    vlr_count: int
    scale: tuple[float, float, float]
    offset: tuple[float, float, float]
    minx: float
    maxx: float
    miny: float
    maxy: float
    minz: float
    maxz: float


def parse_las_header(buf: bytes) -> LasHeader:
    if buf[:4] != b"LASF":
        raise ValueError("not a LAS/LAZ file (missing LASF signature)")

    def u8(o: int) -> int:
        return struct.unpack_from("<B", buf, o)[0]

    def u16(o: int) -> int:
        return struct.unpack_from("<H", buf, o)[0]

    def u32(o: int) -> int:
        return struct.unpack_from("<I", buf, o)[0]

    def u64(o: int) -> int:
        return struct.unpack_from("<Q", buf, o)[0]

    def f64(o: int) -> float:
        return struct.unpack_from("<d", buf, o)[0]

    raw_format = u8(104)

    # LAS 1.4 keeps the authoritative point count in a 64-bit field at 247;
    # older writers only fill the legacy 32-bit field at 107.
    point_count = u64(247) or u32(107)

    return LasHeader(
        version=f"{u8(24)}.{u8(25)}",
        # Bit 7 of the point format byte is the LASzip compression flag.
        point_format=raw_format & 0x3F,
        compressed=bool(raw_format & 0x80),
        point_record_length=u16(105),
        point_count=point_count,
        vlr_count=u32(100),
        scale=(f64(131), f64(139), f64(147)),
        offset=(f64(155), f64(163), f64(171)),
        maxx=f64(179),
        minx=f64(187),
        maxy=f64(195),
        miny=f64(203),
        maxz=f64(211),
        minz=f64(219),
    )


# ============================================================================
# PART 2 -- the "copc info" VLR
#
# The spec pins this down completely: it is ALWAYS the first VLR, so its
# 54-byte VLR header starts at 375 and its 160-byte payload starts at 429.
# No searching, no scanning.
# ============================================================================


@dataclass
class CopcInfo:
    center: tuple[float, float, float]
    halfsize: float
    spacing: float          # <-- the base of the LOD ladder
    root_hier_offset: int
    root_hier_size: int
    gpstime_min: float
    gpstime_max: float

    def spacing_at(self, level: int) -> float:
        """The whole LOD ladder, in one line."""
        return self.spacing / (2 ** level)

    def level_for_resolution(self, resolution: float, max_level: int = 32) -> int:
        """
        The depth cut: descend only while node spacing is still coarser than
        what was asked for. This is exactly what PDAL's `resolution` option
        does during the octree walk.
        """
        level = 0

        while level < max_level and self.spacing_at(level) > resolution:
            level += 1

        return level


def parse_copc_info(buf: bytes) -> CopcInfo:
    user_id = buf[375 + 2: 375 + 2 + 16].rstrip(b"\0")
    record_id = struct.unpack_from("<H", buf, 375 + 2 + 16)[0]

    if not user_id.startswith(b"copc") or record_id != 1:
        raise ValueError(
            "no copc info VLR at offset 375 -- this is a plain LAS/LAZ file, "
            "not a COPC file, so it has no octree and no LOD to query"
        )

    values = struct.unpack_from("<5d2Q2d", buf, 429)

    return CopcInfo(
        center=(values[0], values[1], values[2]),
        halfsize=values[3],
        spacing=values[4],
        root_hier_offset=values[5],
        root_hier_size=values[6],
        gpstime_min=values[7],
        gpstime_max=values[8],
    )


# ============================================================================
# PART 3 -- the octree
#
# A node's bounding box needs NO lookup. It is arithmetic on its key, which is
# why a client can discard an entire subtree without reading a byte of it.
# ============================================================================


@dataclass(frozen=True, order=True)
class VoxelKey:
    level: int
    x: int
    y: int
    z: int

    def __str__(self) -> str:
        return f"{self.level}-{self.x}-{self.y}-{self.z}"

    def children(self) -> Iterator["VoxelKey"]:
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    yield VoxelKey(
                        self.level + 1,
                        self.x * 2 + dx,
                        self.y * 2 + dy,
                        self.z * 2 + dz,
                    )


@dataclass
class Bounds3:
    minx: float
    miny: float
    minz: float
    maxx: float
    maxy: float
    maxz: float

    def intersects_2d(self, xmin, xmax, ymin, ymax) -> bool:
        return not (
            self.maxx < xmin or self.minx > xmax
            or self.maxy < ymin or self.miny > ymax
        )


def bounds_of(key: VoxelKey, info: CopcInfo) -> Bounds3:
    root_min = tuple(c - info.halfsize for c in info.center)

    node_size = (info.halfsize * 2.0) / (2 ** key.level)

    minx = root_min[0] + key.x * node_size
    miny = root_min[1] + key.y * node_size
    minz = root_min[2] + key.z * node_size

    return Bounds3(
        minx, miny, minz,
        minx + node_size, miny + node_size, minz + node_size,
    )


@dataclass
class Node:
    key: VoxelKey
    offset: int
    byte_size: int
    point_count: int


ENTRY_FORMAT = "<4i Q i i"          # level,x,y,z | offset | byteSize | pointCount
ENTRY_SIZE = struct.calcsize(ENTRY_FORMAT)
assert ENTRY_SIZE == 32


def read_hierarchy(
    source: ByteSource,
    info: CopcInfo,
    follow_child_pages: bool = True,
) -> tuple[list[Node], int]:
    """
    Walk the hierarchy pages and return every node that holds points.

    A `point_count` of -1 does not mean "empty" -- it means this entry is a
    POINTER to a further hierarchy page. That is what makes the index itself
    lazy: a client looking at one corner of the cloud never downloads the
    index for the rest of it.
    """
    nodes: list[Node] = []
    pages = 0

    todo = [(info.root_hier_offset, info.root_hier_size)]

    while todo:
        offset, size = todo.pop()

        if size <= 0:
            continue

        page = source.read_range(offset, size)
        pages += 1

        for i in range(len(page) // ENTRY_SIZE):
            level, x, y, z, node_offset, byte_size, point_count = struct.unpack_from(
                ENTRY_FORMAT, page, i * ENTRY_SIZE
            )

            if point_count == -1:
                if follow_child_pages:
                    todo.append((node_offset, byte_size))
            elif point_count > 0:
                nodes.append(
                    Node(VoxelKey(level, x, y, z), node_offset, byte_size, point_count)
                )

    return nodes, pages


# ============================================================================
# PART 4 -- reporting
# ============================================================================


def print_header(header: LasHeader, source: ByteSource) -> None:
    print("\n=== LAS HEADER ===============================================")
    print(f"Source               : {source.name}")
    print(f"LAS version          : {header.version}")
    print(
        f"Point record format  : {header.point_format}"
        f"  ({'LAZ compressed' if header.compressed else 'uncompressed LAS'})"
    )
    print(f"Point record length  : {header.point_record_length} bytes")
    print(f"Number of points     : {header.point_count:,}")
    print(f"VLR count            : {header.vlr_count}")
    print(f"File size            : {source.size:,} bytes "
          f"({source.size / 1024 / 1024:.2f} MB)")
    print(f"Scale                : {header.scale[0]:g}, {header.scale[1]:g}, "
          f"{header.scale[2]:g}")
    print(f"Offset               : {header.offset[0]:.3f}, "
          f"{header.offset[1]:.3f}, {header.offset[2]:.3f}")
    print(f"Bounding box X       : {header.minx:.3f} ... {header.maxx:.3f}"
          f"   ({header.maxx - header.minx:.3f} wide)")
    print(f"Bounding box Y       : {header.miny:.3f} ... {header.maxy:.3f}"
          f"   ({header.maxy - header.miny:.3f} wide)")
    print(f"Bounding box Z       : {header.minz:.3f} ... {header.maxz:.3f}"
          f"   ({header.maxz - header.minz:.3f} wide)")
    print("\n-> All of the above came from the first 375 bytes.")


def print_copc_info(info: CopcInfo, levels: int = 10) -> None:
    print("\n=== COPC INFO VLR ============================================")
    print(f"Root cube center     : {info.center[0]:.4f}, {info.center[1]:.4f}, "
          f"{info.center[2]:.4f}")
    print(f"Root cube halfsize   : {info.halfsize:.4f}")
    print(f"Root cube extent     : {info.center[0] - info.halfsize:.4f} ... "
          f"{info.center[0] + info.halfsize:.4f}  (cubic in X, Y and Z)")
    print(f"Root node spacing    : {info.spacing:.4f}   "
          f"<-- the base of the LOD ladder")
    print(f"Hierarchy at         : offset {info.root_hier_offset:,}, "
          f"{info.root_hier_size:,} bytes")

    print("\nThe LOD ladder is fully determined by that one spacing value:\n")
    print("  level      spacing")
    print("  -----    ---------")

    for level in range(levels + 1):
        print(f"  {level:5d}    {info.spacing_at(level):9.4f}")

    print("\n(No viewer, no library and no renderer chooses these. The")
    print(" COPC WRITER chose them when the file was created.)")


def print_levels(nodes: list[Node], info: CopcInfo, header: LasHeader,
                 pages: int) -> None:
    per_level: dict[int, list[int]] = {}

    for node in nodes:
        stats = per_level.setdefault(node.key.level, [0, 0, 0])
        stats[0] += 1
        stats[1] += node.point_count
        stats[2] += node.byte_size

    print("\n=== OCTREE CONTENTS ==========================================")
    print(f"Hierarchy pages read : {pages}")
    print(f"Nodes with points    : {len(nodes)}")

    print("\n level   nodes       points    spacing   avg pts/node   cumulative")
    print(" -----   -----   ----------   --------   ------------   ----------")

    cumulative = 0

    for level in sorted(per_level):
        count, points, _ = per_level[level]
        cumulative += points

        print(f"{level:6d}  {count:6d}   {points:10,}   "
              f"{info.spacing_at(level):8.4f}   {points // max(count, 1):12,}   "
              f"{cumulative:10,}")

    print(f"\nTotal points in octree : {cumulative:,}")
    print(f"Header point count     : {header.point_count:,}")

    if cumulative == header.point_count:
        print("-> they match, so every point is accounted for by exactly one node.")

    print("\nRead the `cumulative` column as the LOD pyramid: drawing level 0")
    print("alone costs the first row; drawing levels 0..d costs row d. Nodes")
    print("are ADDITIVE -- descending a level never invalidates what you")
    print("already drew, it only refines it.")


def print_tree(nodes: list[Node], info: CopcInfo) -> None:
    print("\n=== NODE LIST ================================================")
    print(" key (l-x-y-z)         points      bytes    X range                "
          "Y range")

    for node in sorted(nodes, key=lambda n: n.key):
        b = bounds_of(node.key, info)
        indent = "  " * node.key.level

        print(f"{indent}{str(node.key):<{20 - len(indent)}}"
              f"{node.point_count:9,} {node.byte_size:10,}   "
              f"{b.minx:10.1f}..{b.maxx:10.1f} {b.miny:11.1f}..{b.maxy:11.1f}")


# ============================================================================
# PART 5 -- costing a query, without running it
#
# Two independent prunings, both decided from the index alone:
#
#     bounds     -> prune in SPACE  (node AABB vs the query box)
#     resolution -> prune in DEPTH  (spacing/2^level vs requested spacing)
# ============================================================================


@dataclass
class QueryCost:
    max_level: int | None
    nodes: int = 0
    points: int = 0
    data_bytes: int = 0
    culled_by_bounds: int = 0
    culled_by_depth: int = 0
    hits: list[Node] = field(default_factory=list)


def cost_query(nodes: list[Node], info: CopcInfo,
               box: tuple[float, float, float, float] | None,
               resolution: float | None) -> QueryCost:
    max_level = info.level_for_resolution(resolution) if resolution else None

    cost = QueryCost(max_level=max_level)

    for node in nodes:
        if max_level is not None and node.key.level > max_level:
            cost.culled_by_depth += 1
            continue

        if box is not None and not bounds_of(node.key, info).intersects_2d(*box):
            cost.culled_by_bounds += 1
            continue

        cost.nodes += 1
        cost.points += node.point_count
        cost.data_bytes += node.byte_size
        cost.hits.append(node)

    return cost


def print_query(cost: QueryCost, nodes: list[Node], info: CopcInfo,
                header: LasHeader, source: ByteSource,
                box: tuple[float, float, float, float],
                resolution: float | None) -> None:
    print("\n=== SIMULATED QUERY ==========================================")
    print(f"bounds     : ([{box[0]:.3f},{box[1]:.3f}],[{box[2]:.3f},{box[3]:.3f}])")

    if resolution:
        print(f"resolution : {resolution:g}")
        print(f"-> deepest level needed : {cost.max_level}  "
              f"(spacing {info.spacing_at(cost.max_level):.4f})")
    else:
        print("resolution : (none -- full available detail)")

    print(f"\nNodes touched        : {cost.nodes} of {len(nodes)}")
    print(f"  culled by bounds   : {cost.culled_by_bounds}")
    print(f"  culled by depth    : {cost.culled_by_depth}")

    pct_points = 100.0 * cost.points / max(header.point_count, 1)
    pct_bytes = 100.0 * cost.data_bytes / max(source.size, 1)

    print(f"\nPoints in those nodes: {cost.points:,} of {header.point_count:,}"
          f"   ({pct_points:.2f} %)")
    print(f"Bytes to read        : {cost.data_bytes:,} of {source.size:,}"
          f"   ({pct_bytes:.2f} %)")
    print(f"Range requests       : {cost.nodes + 2}"
          f"   (header + root hierarchy page + one per node)")

    print("\nCAREFUL: `points in those nodes` is an UPPER BOUND on what a")
    print("reader hands back, not the answer. A node is the smallest unit of")
    print("I/O, so you always decompress whole nodes -- but PDAL and laspy")
    print("then CROP the result to your box. Expect fewer points back, and")
    print("the gap to be large whenever the box is small next to a node.")
    print("\nThe `bytes to read` figure is the one that is exact, and it is")
    print("the one that matters: it is what leaves the disk or the network.")
    print("We computed all of it from the index alone -- no point data was")
    print("decompressed to find out.")


def print_ladder(nodes: list[Node], info: CopcInfo, header: LasHeader,
                 source: ByteSource,
                 box: tuple[float, float, float, float] | None) -> None:
    """Hold the bounds still and move only the resolution."""
    print("\n=== LOD LADDER ===============================================")

    if box:
        print(f"bounds fixed at ([{box[0]:.3f},{box[1]:.3f}],"
              f"[{box[2]:.3f},{box[3]:.3f}]); only `resolution` changes.\n")
    else:
        print("whole extent; only `resolution` changes.\n")

    print("(`points` counts whole nodes, before the reader crops to the box.)\n")
    print(" resolution    level    node points    % of file        bytes   % of file")
    print(" ----------    -----   ------------   ----------   ----------   ---------")

    max_level = max((n.key.level for n in nodes), default=0)

    for level in range(max_level + 1):
        # Ask for exactly the spacing of this level.
        resolution = info.spacing_at(level)
        cost = cost_query(nodes, info, box, resolution)

        print(f" {resolution:10.4f}   {cost.max_level:6d}   {cost.points:12,}   "
              f"{100.0 * cost.points / max(header.point_count, 1):9.2f} %   "
              f"{cost.data_bytes:10,}   "
              f"{100.0 * cost.data_bytes / max(source.size, 1):8.2f} %")

    full = cost_query(nodes, info, box, None)

    print(f" {'(none)':>10}   {'all':>6}   {full.points:12,}   "
          f"{100.0 * full.points / max(header.point_count, 1):9.2f} %   "
          f"{full.data_bytes:10,}   "
          f"{100.0 * full.data_bytes / max(source.size, 1):8.2f} %")

    print("\nOne option changed between those rows. The file, the bounds and")
    print("the code were identical.")


# ============================================================================
# PART 6 -- optionally, actually load the points
#
# Everything above is index arithmetic. This is the one function that pulls
# real point data, so you can check the prediction against reality.
#
# laspy's CopcReader does the same octree walk this script simulates, and is
# the shortest route from "a COPC file" to "a numpy array" in Python.
# ============================================================================


def load_points(path_or_url: str,
                box: tuple[float, float, float, float],
                resolution: float | None,
                minz: float,
                maxz: float):
    try:
        import laspy
        from laspy.copc import CopcReader, Bounds
    except ImportError:
        print("\n--load needs laspy. Install it with:\n")
        print("    conda install -c conda-forge laspy lazrs\n")
        print("or create the env in python/environment.yml.")
        return None

    import numpy as np

    print("\n=== ACTUAL LOAD (laspy) ======================================")

    query_bounds = Bounds(
        mins=np.asarray([box[0], box[2], minz]),
        maxs=np.asarray([box[1], box[3], maxz]),
    )

    opener = (
        CopcReader.open(path_or_url)
        if path_or_url.startswith(("http://", "https://"))
        else CopcReader.open(open(path_or_url, "rb"))
    )

    with opener as reader:
        points = reader.query(bounds=query_bounds, resolution=resolution)

    print(f"laspy returned       : {len(points):,} points")
    print("   (cropped to the box -- compare with `points in those nodes`")
    print("    above, which counts the whole nodes that had to be read)")

    if len(points):
        xyz = np.stack([points.x, points.y, points.z], axis=1)

        print(f"actual X range       : {xyz[:, 0].min():.3f} ... "
              f"{xyz[:, 0].max():.3f}")
        print(f"actual Y range       : {xyz[:, 1].min():.3f} ... "
              f"{xyz[:, 1].max():.3f}")
        print(f"actual Z range       : {xyz[:, 2].min():.3f} ... "
              f"{xyz[:, 2].max():.3f}")
        print(f"numpy array          : {xyz.shape}, "
              f"{xyz.nbytes / 1024 / 1024:.2f} MB")

    print("\n-> laspy walks the same octree with the same two rules. The")
    print("   node-level figures above are what it had to READ; this is what")
    print("   it had to KEEP.")

    return points


# ============================================================================
# MAIN
# ============================================================================


def parse_box(text: str) -> tuple[float, float, float, float]:
    parts = [float(v) for v in text.split(",")]

    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected xmin,xmax,ymin,ymax")

    return parts[0], parts[1], parts[2], parts[3]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read a COPC file's octree index -- locally or over HTTP -- "
                    "without decompressing any points.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("EXAMPLES")[-1],
    )

    parser.add_argument("source", help="path to a .copc.laz file, or an http(s) URL")
    parser.add_argument("--tree", action="store_true",
                        help="list every node with its bounds")
    parser.add_argument("--bounds", type=parse_box, metavar="xmin,xmax,ymin,ymax",
                        help="simulate a spatial query")
    parser.add_argument("--resolution", type=float, metavar="R",
                        help="target point spacing for that query")
    parser.add_argument("--ladder", action="store_true",
                        help="cost the same bounds at every level of detail")
    parser.add_argument("--load", action="store_true",
                        help="actually fetch the points with laspy and compare")
    parser.add_argument("--json", action="store_true",
                        help="emit machine-readable output instead")

    args = parser.parse_args()

    try:
        source = open_source(args.source)
    except Exception as exc:
        print(f"ERROR: cannot open {args.source}: {exc}", file=sys.stderr)
        return 1

    try:
        # One read covers the header AND the copc info VLR, because the spec
        # fixes the VLR's position. Over HTTP this is a single 589-byte GET.
        prefix = source.read_range(0, 589)

        if len(prefix) < 589:
            raise ValueError("file too short to be a COPC file")

        header = parse_las_header(prefix)
        info = parse_copc_info(prefix)

        nodes, pages = read_hierarchy(source, info)

        if args.json:
            print(json.dumps({
                "source": source.name,
                "size": source.size,
                "point_count": header.point_count,
                "bounds": [header.minx, header.maxx, header.miny,
                           header.maxy, header.minz, header.maxz],
                "root_spacing": info.spacing,
                "levels": {
                    str(level): {
                        "spacing": info.spacing_at(level),
                        "nodes": sum(1 for n in nodes if n.key.level == level),
                        "points": sum(n.point_count for n in nodes
                                      if n.key.level == level),
                    }
                    for level in sorted({n.key.level for n in nodes})
                },
                "index_bytes_read": source.bytes_read,
                "index_requests": source.requests,
            }, indent=2))
            return 0

        print_header(header, source)
        print_copc_info(info)
        print_levels(nodes, info, header, pages)

        if args.tree:
            print_tree(nodes, info)

        if args.ladder:
            print_ladder(nodes, info, header, source, args.bounds)

        if args.bounds:
            cost = cost_query(nodes, info, args.bounds, args.resolution)
            print_query(cost, nodes, info, header, source, args.bounds,
                        args.resolution)
        elif not args.ladder:
            print("\n(Pass --bounds xmin,xmax,ymin,ymax [--resolution R] to see")
            print(" how little of the file a partial load would touch, or")
            print(" --ladder for every level at once.)")

        # --------------------------------------------------------------------
        # The receipt. Everything above was derived from this many bytes.
        # --------------------------------------------------------------------
        print("\n=== WHAT THIS SCRIPT ACTUALLY READ ===========================")
        print(f"Requests             : {source.requests}")
        print(f"Bytes read           : {source.bytes_read:,} of "
              f"{source.size:,}  "
              f"({100.0 * source.bytes_read / max(source.size, 1):.4f} %)")
        print("Points decompressed  : 0")

        if isinstance(source, HttpSource):
            print("\n-> That was over HTTP, against a static file. No server")
            print("   software, no database, no preprocessing step -- just")
            print("   Range requests. This is what 'cloud optimized' means.")

        if args.load:
            if not args.bounds:
                print("\n--load needs --bounds", file=sys.stderr)
                return 1

            load_points(args.source, args.bounds, args.resolution,
                        header.minz, header.maxz)

        print()

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        if isinstance(source, FileSource):
            source.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
