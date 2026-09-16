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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// ----------------------------------------------------------------------------
// Little helper: read a POD value from a byte buffer at a given offset.
// ----------------------------------------------------------------------------
template <typename T>
static T readAt(const std::vector<char> &buf, std::size_t offset)
{
    T value{};
    std::memcpy(&value, buf.data() + offset, sizeof(T));
    return value;
}

// ============================================================================
// PART 1  --  The LAS public header block
//
// 375 bytes, fixed layout, LAS 1.4. Every offset below is straight out of the
// ASPRS spec. This is all you need to know how big the cloud is and where it
// lives in space -- WITHOUT reading any points.
// ============================================================================

struct LasHeader
{
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    uint16_t headerSize = 0;
    uint32_t offsetToPointData = 0;
    uint32_t numberOfVlrs = 0;
    uint8_t pointDataRecordFormat = 0;
    uint16_t pointDataRecordLength = 0;
    uint64_t numberOfPointRecords = 0;

    double scaleX = 0, scaleY = 0, scaleZ = 0;
    double offsetX = 0, offsetY = 0, offsetZ = 0;
    double minX = 0, maxX = 0;
    double minY = 0, maxY = 0;
    double minZ = 0, maxZ = 0;
};

static LasHeader parseLasHeader(const std::vector<char> &buf)
{
    if (buf.size() < 375)
        throw std::runtime_error("file shorter than a LAS 1.4 header");

    if (std::strncmp(buf.data(), "LASF", 4) != 0)
        throw std::runtime_error("not a LAS/LAZ file (missing LASF signature)");

    LasHeader h;

    h.versionMajor = readAt<uint8_t>(buf, 24);
    h.versionMinor = readAt<uint8_t>(buf, 25);
    h.headerSize = readAt<uint16_t>(buf, 94);
    h.offsetToPointData = readAt<uint32_t>(buf, 96);
    h.numberOfVlrs = readAt<uint32_t>(buf, 100);

    // The high bit of the point format flags LAZ compression.
    h.pointDataRecordFormat = readAt<uint8_t>(buf, 104);
    h.pointDataRecordLength = readAt<uint16_t>(buf, 105);

    h.scaleX = readAt<double>(buf, 131);
    h.scaleY = readAt<double>(buf, 139);
    h.scaleZ = readAt<double>(buf, 147);
    h.offsetX = readAt<double>(buf, 155);
    h.offsetY = readAt<double>(buf, 163);
    h.offsetZ = readAt<double>(buf, 171);

    h.maxX = readAt<double>(buf, 179);
    h.minX = readAt<double>(buf, 187);
    h.maxY = readAt<double>(buf, 195);
    h.minY = readAt<double>(buf, 203);
    h.maxZ = readAt<double>(buf, 211);
    h.minZ = readAt<double>(buf, 219);

    // LAS 1.4 moved the authoritative count to a 64-bit field at 247. Older
    // writers only fill the legacy 32-bit field at 107.
    h.numberOfPointRecords = readAt<uint64_t>(buf, 247);

    if (h.numberOfPointRecords == 0)
        h.numberOfPointRecords = readAt<uint32_t>(buf, 107);

    return h;
}

// ============================================================================
// PART 2  --  The "copc info" VLR
//
// The COPC spec pins this down completely: it is ALWAYS the first VLR, so its
// 54-byte VLR header starts at 375 and its 160 bytes of payload start at 429.
// No searching required.
//
//      offset 429   double center_x
//      offset 437   double center_y
//      offset 445   double center_z
//      offset 453   double halfsize        <-- root cube half edge length
//      offset 461   double spacing         <-- ROOT NODE POINT SPACING
//      offset 469   uint64 root_hier_offset
//      offset 477   uint64 root_hier_size
//      offset 485   double gpstime_minimum
//      offset 493   double gpstime_maximum
//      offset 501   uint64 reserved[11]
//
// `spacing` is the single number that defines the whole LOD ladder:
//
//      spacing(level d) = spacing / 2^d
//
// ============================================================================

struct CopcInfo
{
    double centerX = 0, centerY = 0, centerZ = 0;
    double halfsize = 0;
    double spacing = 0;
    uint64_t rootHierOffset = 0;
    uint64_t rootHierSize = 0;
    double gpsTimeMin = 0, gpsTimeMax = 0;
};

static CopcInfo parseCopcInfo(const std::vector<char> &buf)
{
    if (buf.size() < 589)
        throw std::runtime_error("file too short to contain a copc info VLR");

    // Sanity-check the VLR header sitting at 375 before trusting the payload.
    char userId[17] = {0};
    std::memcpy(userId, buf.data() + 375 + 2, 16);

    const uint16_t recordId = readAt<uint16_t>(buf, 375 + 2 + 16);

    if (std::strncmp(userId, "copc", 4) != 0 || recordId != 1)
    {
        throw std::runtime_error(
            "no copc info VLR at offset 375 -- this is a plain LAS/LAZ file, "
            "not a COPC file (so it has no octree and no LOD to query)");
    }

    CopcInfo info;

    info.centerX = readAt<double>(buf, 429);
    info.centerY = readAt<double>(buf, 437);
    info.centerZ = readAt<double>(buf, 445);
    info.halfsize = readAt<double>(buf, 453);
    info.spacing = readAt<double>(buf, 461);
    info.rootHierOffset = readAt<uint64_t>(buf, 469);
    info.rootHierSize = readAt<uint64_t>(buf, 477);
    info.gpsTimeMin = readAt<double>(buf, 485);
    info.gpsTimeMax = readAt<double>(buf, 493);

    return info;
}

// ============================================================================
// PART 3  --  The octree itself
//
// A node is named by a VoxelKey (level, x, y, z). Its bounding box needs NO
// lookup -- it is pure arithmetic from the root cube:
//
//      nodeSize = (halfsize * 2) / 2^level
//      min      = rootMin + key.{x,y,z} * nodeSize
//
// This is why a client can cull a whole subtree without reading anything.
// ============================================================================

struct VoxelKey
{
    int32_t level = 0, x = 0, y = 0, z = 0;

    bool operator<(const VoxelKey &o) const
    {
        if (level != o.level) return level < o.level;
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }

    std::string str() const
    {
        return std::to_string(level) + "-" + std::to_string(x) + "-" +
               std::to_string(y) + "-" + std::to_string(z);
    }
};

struct Bounds3
{
    double minx, miny, minz, maxx, maxy, maxz;
};

static Bounds3 boundsOf(const VoxelKey &k, const CopcInfo &info)
{
    const double rootMinX = info.centerX - info.halfsize;
    const double rootMinY = info.centerY - info.halfsize;
    const double rootMinZ = info.centerZ - info.halfsize;

    const double nodeSize =
        (info.halfsize * 2.0) / static_cast<double>(1ULL << k.level);

    Bounds3 b;

    b.minx = rootMinX + k.x * nodeSize;
    b.miny = rootMinY + k.y * nodeSize;
    b.minz = rootMinZ + k.z * nodeSize;
    b.maxx = b.minx + nodeSize;
    b.maxy = b.miny + nodeSize;
    b.maxz = b.minz + nodeSize;

    return b;
}

// The LOD ladder, in one line.
static double spacingAtLevel(const CopcInfo &info, int level)
{
    return info.spacing / static_cast<double>(1ULL << level);
}

// ----------------------------------------------------------------------------
// A hierarchy entry is 32 bytes:
//
//      int32  level, x, y, z     the VoxelKey
//      uint64 offset             absolute byte offset of this node's LAZ chunk
//      int32  byteSize           compressed size of that chunk
//      int32  pointCount         >0  a real node with this many points
//                                 0  the node exists but is empty
//                                -1  this key is a POINTER to a child
//                                    hierarchy page living at offset/byteSize
//
// That -1 case is what makes the INDEX itself lazy: a client that only ever
// looks at one corner of the cloud never downloads the index for the rest.
// ----------------------------------------------------------------------------

struct Node
{
    VoxelKey key;
    uint64_t offset = 0;
    int32_t byteSize = 0;
    int32_t pointCount = 0;
};

// Read one hierarchy page from the file and append its real nodes to `nodes`.
// Child pages are followed recursively -- which is where the extra reads that
// a real streaming client would defer show up.
static void readHierarchyPage(std::ifstream &file,
                              uint64_t pageOffset,
                              uint64_t pageSize,
                              std::vector<Node> &nodes,
                              std::size_t &pagesRead)
{
    if (pageSize == 0)
        return;

    ++pagesRead;

    std::vector<char> page(static_cast<std::size_t>(pageSize));

    file.seekg(static_cast<std::streamoff>(pageOffset));
    file.read(page.data(), static_cast<std::streamsize>(pageSize));

    if (!file)
        throw std::runtime_error("failed to read hierarchy page at offset " +
                                 std::to_string(pageOffset));

    const std::size_t entryCount = page.size() / 32;

    // Collect child pages first, then recurse, so we do not interleave seeks
    // with the parsing of the page we are currently holding.
    std::vector<std::pair<uint64_t, uint64_t>> childPages;

    for (std::size_t i = 0; i < entryCount; ++i)
    {
        const std::size_t base = i * 32;

        Node n;
        n.key.level = readAt<int32_t>(page, base + 0);
        n.key.x = readAt<int32_t>(page, base + 4);
        n.key.y = readAt<int32_t>(page, base + 8);
        n.key.z = readAt<int32_t>(page, base + 12);
        n.offset = readAt<uint64_t>(page, base + 16);
        n.byteSize = readAt<int32_t>(page, base + 24);
        n.pointCount = readAt<int32_t>(page, base + 28);

        if (n.pointCount == -1)
            childPages.emplace_back(n.offset,
                                    static_cast<uint64_t>(n.byteSize));
        else if (n.pointCount > 0)
            nodes.push_back(n);
    }

    for (const auto &[off, size] : childPages)
        readHierarchyPage(file, off, size, nodes, pagesRead);
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

    // Bit 7 of the point format byte is the LASzip compression flag.
    const bool compressed = (h.pointDataRecordFormat & 0x80) != 0;

    std::cout << "Point record format  : " << int(h.pointDataRecordFormat & 0x3F)
              << (compressed ? "  (LAZ compressed)" : "  (uncompressed LAS)")
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
        std::ifstream file(path, std::ios::binary);

        if (!file)
            throw std::runtime_error("cannot open " + path);

        file.seekg(0, std::ios::end);
        const uint64_t fileSize = static_cast<uint64_t>(file.tellg());
        file.seekg(0);

        // Read only the fixed prefix that holds the header and the copc info
        // VLR. A real network client would do this as a single 589-byte range
        // request.
        std::vector<char> prefix(589);
        file.read(prefix.data(), 589);

        if (!file)
            throw std::runtime_error("file too short to be a COPC file");

        const LasHeader header = parseLasHeader(prefix);
        const CopcInfo info = parseCopcInfo(prefix);

        printHeader(header, fileSize);
        printCopcInfo(info);

        // Now the index. One seek, one read (plus any child pages).
        std::vector<Node> nodes;
        std::size_t pagesRead = 0;

        readHierarchyPage(file, info.rootHierOffset, info.rootHierSize, nodes,
                          pagesRead);

        printLevelStatistics(nodes, info, header.numberOfPointRecords, pagesRead);

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
