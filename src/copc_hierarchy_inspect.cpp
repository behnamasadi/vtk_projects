// ============================================================================
// copc_hierarchy_inspect
//
// "Where does the level of detail actually come from?"
//
// The answer is: it is BAKED INTO THE FILE. A COPC file carries its own octree
// index. Nothing in VTK, nothing in your viewer, and nothing in PDAL invents
// the levels -- they were decided once, by the writer, and they sit in two
// VLRs that you can read with plain std::ifstream.
//
// This program does exactly that, with NO dependencies at all: no PDAL, no
// VTK, no LAZ decoder. It never decompresses a single point. It only reads:
//
//      bytes 0..374    the LAS public header block
//      bytes 429..588  the "copc info" VLR  (always at this fixed offset)
//      the hierarchy pages it points to
//
// and from that it can tell you, before loading anything:
//
//      * how deep the octree is
//      * how many nodes and points live at each level
//      * the point SPACING at each level  <-- this IS the LOD ladder
//      * which nodes a given bounds + resolution query would touch
//      * how many bytes that query would read vs. the size of the whole file
//
// The parsing lives in src/copc_index.hpp so that copc_camera_streaming can
// use the same index to pick a level of detail before it queries. This file is
// the reporting front end for it.
//
// Build (no CMake needed):
//      g++ -std=c++17 -O2 -o copc_hierarchy_inspect src/copc_hierarchy_inspect.cpp
//
// Usage:
//      ./copc_hierarchy_inspect file.copc.laz
//      ./copc_hierarchy_inspect file.copc.laz --tree
//      ./copc_hierarchy_inspect file.copc.laz --bounds 100,200,300,400 --resolution 5
//
// Assumes a little-endian host (x86-64, arm64). LAS is little-endian on disk.
// ============================================================================

#include "copc_index.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using copc::Bounds3;
using copc::CopcInfo;
using copc::Index;
using copc::LasHeader;
using copc::Node;
using copc::QueryCost;
using copc::VoxelKey;
using copc::boundsOf;

// Kept as a free function because the report prints it in a dozen places.
static double spacingAtLevel(const CopcInfo &info, int level)
{
    return info.spacingAtLevel(level);
}

// ============================================================================
// PART 4  --  Reporting
// ============================================================================

static void printHeader(const LasHeader &h, uint64_t fileSize)
{
    std::cout << "\n=== LAS HEADER ===============================================\n";
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "LAS version          : " << int(h.versionMajor) << "."
              << int(h.versionMinor) << "\n";

    // parseLasHeader has already split the LASzip compression bit out of the
    // point format byte.
    std::cout << "Point record format  : " << int(h.pointDataRecordFormat)
              << (h.compressed ? "  (LAZ compressed)" : "  (uncompressed LAS)")
              << "\n";

    std::cout << "Point record length  : " << h.pointDataRecordLength << " bytes\n";
    std::cout << "Number of points     : " << h.numberOfPointRecords << "\n";
    std::cout << "VLR count            : " << h.numberOfVlrs << "\n";
    std::cout << "File size            : " << fileSize << " bytes ("
              << (double(fileSize) / (1024.0 * 1024.0)) << " MB)\n";

    // Scales are typically 1e-2 .. 1e-9, so the default precision hides them.
    std::cout << "Scale                : " << std::setprecision(9) << h.scaleX
              << ", " << h.scaleY << ", " << h.scaleZ << std::setprecision(3)
              << "\n";
    std::cout << "Offset               : " << h.offsetX << ", " << h.offsetY
              << ", " << h.offsetZ << "\n";

    std::cout << "Bounding box X       : " << h.minX << " ... " << h.maxX
              << "   (" << (h.maxX - h.minX) << " wide)\n";
    std::cout << "Bounding box Y       : " << h.minY << " ... " << h.maxY
              << "   (" << (h.maxY - h.minY) << " wide)\n";
    std::cout << "Bounding box Z       : " << h.minZ << " ... " << h.maxZ
              << "   (" << (h.maxZ - h.minZ) << " wide)\n";

    // The whole point: we now know the extent of the cloud having read 375
    // bytes. Deciding what to load costs nothing.
    std::cout << "\n-> All of the above came from the first 375 bytes.\n";
}

static void printCopcInfo(const CopcInfo &info)
{
    std::cout << "\n=== COPC INFO VLR ============================================\n";
    std::cout << std::fixed << std::setprecision(4);

    std::cout << "Root cube center     : " << info.centerX << ", " << info.centerY
              << ", " << info.centerZ << "\n";
    std::cout << "Root cube halfsize   : " << info.halfsize << "\n";
    std::cout << "Root cube extent     : "
              << (info.centerX - info.halfsize) << " ... "
              << (info.centerX + info.halfsize) << "  (cubic in X, Y and Z)\n";

    std::cout << "Root node spacing    : " << info.spacing
              << "   <-- the base of the LOD ladder\n";

    std::cout << "Hierarchy at         : offset " << info.rootHierOffset
              << ", " << info.rootHierSize << " bytes\n";

    std::cout << "\nThe LOD ladder is fully determined by that one spacing value:\n";
    std::cout << "\n  level    spacing      a query with `resolution` >= this\n";
    std::cout << "                          stops descending here\n";
    std::cout << "  -----    ---------    ------------------------------------\n";

    for (int level = 0; level <= 12; ++level)
    {
        std::cout << "  " << std::setw(5) << level << "    " << std::setw(9)
                  << std::setprecision(4) << spacingAtLevel(info, level) << "\n";
    }

    std::cout << "\n(No viewer, no library and no renderer chooses these. The\n"
                 " COPC WRITER chose them when the file was created.)\n";
}

static void printLevelStatistics(const std::vector<Node> &nodes,
                                 const CopcInfo &info,
                                 uint64_t totalPoints,
                                 std::size_t pagesRead)
{
    std::map<int32_t, std::pair<uint64_t, uint64_t>> perLevel; // level -> (nodes, points)
    std::map<int32_t, uint64_t> bytesPerLevel;

    for (const Node &n : nodes)
    {
        perLevel[n.key.level].first += 1;
        perLevel[n.key.level].second += static_cast<uint64_t>(n.pointCount);
        bytesPerLevel[n.key.level] += static_cast<uint64_t>(n.byteSize);
    }

    std::cout << "\n=== OCTREE CONTENTS ==========================================\n";
    std::cout << "Hierarchy pages read : " << pagesRead << "\n";
    std::cout << "Nodes with points    : " << nodes.size() << "\n";

    std::cout << "\n level   nodes      points    spacing   avg pts/node   "
                 "cumulative pts\n";
    std::cout << " -----   -----   ---------   --------   ------------   "
                 "--------------\n";

    uint64_t cumulative = 0;

    for (const auto &[level, counts] : perLevel)
    {
        cumulative += counts.second;

        std::cout << std::setw(6) << level << "  " << std::setw(6) << counts.first
                  << "  " << std::setw(10) << counts.second << "   "
                  << std::fixed << std::setprecision(4) << std::setw(8)
                  << spacingAtLevel(info, level) << "   " << std::setw(12)
                  << (counts.second / std::max<uint64_t>(counts.first, 1))
                  << "   " << std::setw(14) << cumulative << "\n";
    }

    std::cout << "\nTotal points in octree : " << cumulative << "\n";
    std::cout << "Header point count     : " << totalPoints << "\n";

    std::cout << "\nRead the `cumulative` column as the LOD pyramid: drawing\n"
                 "level 0 alone costs the first row; drawing levels 0..d costs\n"
                 "row d. Nodes are ADDITIVE -- descending a level never\n"
                 "invalidates what you already drew, it only refines it.\n";
}

static void printTree(const std::vector<Node> &nodes, const CopcInfo &info)
{
    std::vector<Node> sorted = nodes;

    std::sort(sorted.begin(), sorted.end(),
              [](const Node &a, const Node &b) { return a.key < b.key; });

    std::cout << "\n=== NODE LIST ================================================\n";
    std::cout << " key (l-x-y-z)      points     bytes      X range              "
                 "Y range\n";

    for (const Node &n : sorted)
    {
        const Bounds3 b = boundsOf(n.key, info);

        std::cout << std::string(static_cast<std::size_t>(n.key.level) * 2, ' ')
                  << std::left << std::setw(18 - n.key.level * 2) << n.key.str()
                  << std::right << std::setw(8) << n.pointCount << std::setw(10)
                  << n.byteSize << "   " << std::fixed << std::setprecision(1)
                  << std::setw(8) << b.minx << ".." << std::setw(8) << b.maxx
                  << "  " << std::setw(8) << b.miny << ".." << std::setw(8)
                  << b.maxy << "\n";
    }
}

// ============================================================================
// PART 5  --  Simulate a query
//
// This is the whole argument for COPC, made concrete. Given a 2D bounds box
// and a target resolution, walk the node list and answer:
//
//      how many nodes intersect?
//      how many points would come back?
//      how many BYTES would actually be read off disk / off the network?
//
// ...all without decompressing anything.
// ============================================================================

static bool intersects2D(const Bounds3 &b,
                         double xmin, double xmax,
                         double ymin, double ymax)
{
    return !(b.maxx < xmin || b.minx > xmax ||
             b.maxy < ymin || b.miny > ymax);
}

static void simulateQuery(const std::vector<Node> &nodes,
                          const CopcInfo &info,
                          uint64_t fileSize,
                          uint64_t totalPoints,
                          double xmin, double xmax,
                          double ymin, double ymax,
                          double resolution)
{
    std::cout << "\n=== SIMULATED QUERY ==========================================\n";
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "bounds     : ([" << xmin << "," << xmax << "],[" << ymin << ","
              << ymax << "])\n";

    if (resolution > 0.0)
        std::cout << "resolution : " << resolution << "\n";
    else
        std::cout << "resolution : (none -- full available detail)\n";

    // The depth cut: descend only while the node spacing is still coarser than
    // what was asked for. This is what PDAL's `resolution` option does.
    int maxLevel = 1000;

    if (resolution > 0.0)
    {
        maxLevel = 0;
        while (maxLevel < 32 && spacingAtLevel(info, maxLevel) > resolution)
            ++maxLevel;

        std::cout << "-> deepest level needed : " << maxLevel << "  (spacing "
                  << spacingAtLevel(info, maxLevel) << ")\n";
    }

    uint64_t hitNodes = 0, hitPoints = 0, hitBytes = 0;
    uint64_t culledBySpace = 0, culledByDepth = 0;

    for (const Node &n : nodes)
    {
        if (n.key.level > maxLevel)
        {
            ++culledByDepth;
            continue;
        }

        if (!intersects2D(boundsOf(n.key, info), xmin, xmax, ymin, ymax))
        {
            ++culledBySpace;
            continue;
        }

        ++hitNodes;
        hitPoints += static_cast<uint64_t>(n.pointCount);
        hitBytes += static_cast<uint64_t>(n.byteSize);
    }

    std::cout << "\nNodes touched        : " << hitNodes << " of " << nodes.size()
              << "\n";
    std::cout << "  culled by bounds   : " << culledBySpace << "\n";
    std::cout << "  culled by depth    : " << culledByDepth << "\n";

    std::cout << "\nPoints in those nodes: " << hitPoints << " of " << totalPoints;

    if (totalPoints > 0)
        std::cout << "   (" << std::setprecision(2)
                  << (100.0 * double(hitPoints) / double(totalPoints)) << " %)";

    std::cout << "\n";

    std::cout << "Bytes read           : " << hitBytes << " of " << fileSize;

    if (fileSize > 0)
        std::cout << "   (" << std::setprecision(2)
                  << (100.0 * double(hitBytes) / double(fileSize)) << " %)";

    std::cout << "\n";

    std::cout << "Range requests       : " << (hitNodes + 2)
              << "   (header + root hierarchy page + one per node)\n";

    std::cout << "\nCAREFUL: `points in those nodes` is an UPPER BOUND on what a\n"
                 "reader hands back, not the answer. A node is the smallest unit\n"
                 "of I/O, so whole nodes are always decompressed -- but PDAL then\n"
                 "CROPS the result to your box. Expect fewer points back, and the\n"
                 "gap to be large whenever the box is small next to a node.\n"
                 "\nThe `bytes read` figure is the exact one, and it is the one\n"
                 "that matters: it is what leaves the disk or the network. All of\n"
                 "it came from the index -- no point data was decompressed.\n";
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr
            << "usage: " << argv[0] << " <file.copc.laz> [options]\n\n"
            << "  --tree                       list every node with its bounds\n"
            << "  --bounds xmin,xmax,ymin,ymax simulate a spatial query\n"
            << "  --resolution R               target point spacing for that query\n\n"
            << "example:\n"
            << "  " << argv[0]
            << " synthetic.copc.laz --bounds 100,200,300,400 --resolution 5\n";

        return 1;
    }

    const std::string path = argv[1];

    bool showTree = false;
    bool haveBounds = false;
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
    double resolution = 0.0;

    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--tree")
        {
            showTree = true;
        }
        else if (arg == "--bounds" && i + 1 < argc)
        {
            if (std::sscanf(argv[++i], "%lf,%lf,%lf,%lf",
                            &xmin, &xmax, &ymin, &ymax) != 4)
            {
                std::cerr << "--bounds wants xmin,xmax,ymin,ymax\n";
                return 1;
            }
            haveBounds = true;
        }
        else if (arg == "--resolution" && i + 1 < argc)
        {
            resolution = std::atof(argv[++i]);
        }
        else
        {
            std::cerr << "unknown argument: " << arg << "\n";
            return 1;
        }
    }

    try
    {
        // Everything the octree needs, in one call: the 589-byte prefix that
        // holds the LAS header and the copc info VLR, then the hierarchy
        // pages it points at. A network client would do the first part as a
        // single range GET.
        const Index index = copc::openIndex(path);

        const LasHeader &header = index.header;
        const CopcInfo &info = index.info;
        const std::vector<Node> &nodes = index.nodes;
        const uint64_t fileSize = index.fileSize;

        printHeader(header, fileSize);
        printCopcInfo(info);

        printLevelStatistics(nodes, info, header.numberOfPointRecords,
                             index.pagesRead);

        if (showTree)
            printTree(nodes, info);

        if (haveBounds)
        {
            simulateQuery(nodes, info, fileSize, header.numberOfPointRecords,
                          xmin, xmax, ymin, ymax, resolution);
        }
        else
        {
            std::cout << "\n(Pass --bounds xmin,xmax,ymin,ymax [--resolution R] "
                         "to see how\n little of the file a partial load would "
                         "actually touch.)\n";
        }

        std::cout << "\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
