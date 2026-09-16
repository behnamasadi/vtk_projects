// ============================================================================
// copc_index.hpp -- the COPC octree index, with no dependencies.
//
// Header-only. No PDAL, no VTK, no LAZ decoder. This parses the parts of a
// COPC file that describe its octree:
//
//      bytes 0..374      LAS 1.4 public header block  -> extent, point count
//      bytes 429..588    the "copc info" VLR           -> root cube, SPACING
//      the hierarchy pages it points to                -> every node's key,
//                                                         byte range, count
//
// and offers the arithmetic a viewer needs to decide what to load:
//
//      spacingAtLevel()        the LOD ladder
//      levelForResolution()    the depth cut PDAL's `resolution` performs
//      boundsOf()              a node's box, from its key alone
//      costQuery()             what a bounds+resolution query would touch
//      chooseLevelForBudget()  the deepest level that fits a point budget
//
// Two programs use this:
//
//   copc_hierarchy_inspect  prints all of it
//   copc_camera_streaming   uses costQuery/chooseLevelForBudget to pick a
//                           level BEFORE issuing a PDAL query, so that every
//                           camera move costs exactly one read instead of a
//                           guess-and-retry loop
//
// Assumes a little-endian host (x86-64, arm64). LAS is little-endian on disk.
// ============================================================================

#ifndef COPC_INDEX_HPP
#define COPC_INDEX_HPP

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace copc
{

// ----------------------------------------------------------------------------
// Read a POD value from a byte buffer at a given offset.
// ----------------------------------------------------------------------------
template <typename T>
inline T readAt(const std::vector<char> &buf, std::size_t offset)
{
    T value{};
    std::memcpy(&value, buf.data() + offset, sizeof(T));
    return value;
}

// ============================================================================
// The LAS 1.4 public header block: 375 bytes, fixed layout.
// ============================================================================

struct LasHeader
{
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    uint16_t headerSize = 0;
    uint32_t offsetToPointData = 0;
    uint32_t numberOfVlrs = 0;
    uint8_t pointDataRecordFormat = 0;
    bool compressed = false;
    uint16_t pointDataRecordLength = 0;
    uint64_t numberOfPointRecords = 0;

    double scaleX = 0, scaleY = 0, scaleZ = 0;
    double offsetX = 0, offsetY = 0, offsetZ = 0;
    double minX = 0, maxX = 0;
    double minY = 0, maxY = 0;
    double minZ = 0, maxZ = 0;
};

inline LasHeader parseLasHeader(const std::vector<char> &buf)
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

    // Bit 7 of the point format byte is the LASzip compression flag.
    const uint8_t rawFormat = readAt<uint8_t>(buf, 104);

    h.pointDataRecordFormat = rawFormat & 0x3F;
    h.compressed = (rawFormat & 0x80) != 0;
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

    // LAS 1.4 keeps the authoritative count at 247; older writers only fill
    // the legacy 32-bit field at 107.
    h.numberOfPointRecords = readAt<uint64_t>(buf, 247);

    if (h.numberOfPointRecords == 0)
        h.numberOfPointRecords = readAt<uint32_t>(buf, 107);

    return h;
}

// ============================================================================
// The "copc info" VLR. The spec pins it to a fixed position: its 54-byte VLR
// header starts at 375, its 160-byte payload at 429.
// ============================================================================

struct CopcInfo
{
    double centerX = 0, centerY = 0, centerZ = 0;
    double halfsize = 0;
    double spacing = 0;          // <-- the base of the LOD ladder
    uint64_t rootHierOffset = 0;
    uint64_t rootHierSize = 0;
    double gpsTimeMin = 0, gpsTimeMax = 0;

    // The whole LOD ladder, in one line.
    double spacingAtLevel(int level) const
    {
        return spacing / static_cast<double>(1ULL << level);
    }

    // The depth cut: descend only while node spacing is still coarser than
    // what was asked for. This is what PDAL's `resolution` option does.
    int levelForResolution(double resolution, int maxLevel = 32) const
    {
        int level = 0;

        while (level < maxLevel && spacingAtLevel(level) > resolution)
            ++level;

        return level;
    }
};

inline CopcInfo parseCopcInfo(const std::vector<char> &buf)
{
    if (buf.size() < 589)
        throw std::runtime_error("file too short to contain a copc info VLR");

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
// The octree.
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

    bool intersects2D(double xmin, double xmax, double ymin, double ymax) const
    {
        return !(maxx < xmin || minx > xmax || maxy < ymin || miny > ymax);
    }
};

// A node's box needs NO lookup: it is arithmetic on its key. That is what lets
// a client discard an entire subtree without reading a byte of it.
inline Bounds3 boundsOf(const VoxelKey &k, const CopcInfo &info)
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

// ----------------------------------------------------------------------------
// A hierarchy entry is 32 bytes:
//
//      int32  level, x, y, z
//      uint64 offset             absolute byte offset of the node's LAZ chunk
//      int32  byteSize           compressed size of that chunk
//      int32  pointCount         >0  a real node
//                                 0  the node exists but is empty
//                                -1  this key POINTS AT a child hierarchy page
//
// The -1 case makes the index itself lazy: a client that only ever looks at
// one corner never downloads the index for the rest.
// ----------------------------------------------------------------------------

struct Node
{
    VoxelKey key;
    uint64_t offset = 0;
    int32_t byteSize = 0;
    int32_t pointCount = 0;
};

inline void readHierarchyPage(std::ifstream &file,
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

    for (const auto &page_ : childPages)
        readHierarchyPage(file, page_.first, page_.second, nodes, pagesRead);
}

// ============================================================================
// The whole index, in one object.
// ============================================================================

struct Index
{
    LasHeader header;
    CopcInfo info;
    std::vector<Node> nodes;
    std::size_t pagesRead = 0;
    uint64_t fileSize = 0;

    int maxLevel() const
    {
        int level = 0;

        for (const Node &n : nodes)
            level = std::max(level, n.key.level);

        return level;
    }
};

inline Index openIndex(const std::string &path)
{
    std::ifstream file(path, std::ios::binary);

    if (!file)
        throw std::runtime_error("cannot open " + path);

    Index index;

    file.seekg(0, std::ios::end);
    index.fileSize = static_cast<uint64_t>(file.tellg());
    file.seekg(0);

    // One read covers the header AND the copc info VLR, because the spec fixes
    // the VLR's position. Over HTTP this would be a single 589-byte range GET.
    std::vector<char> prefix(589);
    file.read(prefix.data(), 589);

    if (!file)
        throw std::runtime_error("file too short to be a COPC file");

    index.header = parseLasHeader(prefix);
    index.info = parseCopcInfo(prefix);

    readHierarchyPage(file, index.info.rootHierOffset, index.info.rootHierSize,
                      index.nodes, index.pagesRead);

    return index;
}

// ============================================================================
// Costing a query WITHOUT running it.
//
// Two independent prunings, both decided from the index alone:
//
//      bounds     -> prune in SPACE  (node box vs the query box)
//      resolution -> prune in DEPTH  (spacing/2^level vs requested spacing)
// ============================================================================

struct QueryCost
{
    int maxLevel = -1;           // -1 means "no depth limit"
    uint64_t nodes = 0;
    uint64_t points = 0;         // UPPER BOUND -- see the note below
    uint64_t dataBytes = 0;      // exact
    uint64_t culledByBounds = 0;
    uint64_t culledByDepth = 0;
};

// NOTE on `points`: this counts every point inside the nodes that would have to
// be decompressed. A reader (PDAL, laspy) then CROPS the result to the query
// box, so it hands back fewer. `points` is therefore an upper bound, and
// `dataBytes` is the exact figure -- it is what actually leaves the disk or the
// network. The upper bound is still the useful one for budgeting, because a
// level that fits the budget by this measure is guaranteed to fit for real.
inline QueryCost costQuery(const std::vector<Node> &nodes,
                           const CopcInfo &info,
                           double xmin, double xmax,
                           double ymin, double ymax,
                           double resolution)
{
    QueryCost cost;

    cost.maxLevel =
        (resolution > 0.0) ? info.levelForResolution(resolution) : -1;

    for (const Node &node : nodes)
    {
        if (cost.maxLevel >= 0 && node.key.level > cost.maxLevel)
        {
            ++cost.culledByDepth;
            continue;
        }

        if (!boundsOf(node.key, info).intersects2D(xmin, xmax, ymin, ymax))
        {
            ++cost.culledByBounds;
            continue;
        }

        ++cost.nodes;
        cost.points += static_cast<uint64_t>(node.pointCount);
        cost.dataBytes += static_cast<uint64_t>(node.byteSize);
    }

    return cost;
}

// ============================================================================
// Budgeting from the index.
//
// Given a box and the deepest level the camera would like, return the deepest
// level whose cost fits a point budget.
//
// This is the whole reason to keep the hierarchy client-side. The naive
// alternative -- query, notice the result is too big, double the resolution,
// query again -- costs a full read per guess, and wastes one entirely whenever
// doubling the resolution does not happen to change the depth. Here the answer
// is pure arithmetic over an index we already hold, so the caller issues
// exactly ONE query, at a level it already knows will fit.
//
// Conservative by construction: QueryCost::points bounds the node contents, not
// the cropped result, so this can pick a level coarser than strictly necessary.
// Erring toward too few points is the right default -- the alternative is a
// dropped frame.
// ============================================================================

struct BudgetChoice
{
    int wantedLevel = 0;         // what the camera asked for
    int chosenLevel = 0;         // what fits the budget
    double resolution = 0.0;     // spacing of the chosen level
    QueryCost cost;              // cost of the chosen level
};

inline BudgetChoice chooseLevelForBudget(const std::vector<Node> &nodes,
                                         const CopcInfo &info,
                                         double xmin, double xmax,
                                         double ymin, double ymax,
                                         double wantedResolution,
                                         uint64_t pointBudget)
{
    BudgetChoice choice;

    choice.wantedLevel = info.levelForResolution(wantedResolution);

    for (int level = choice.wantedLevel; level >= 0; --level)
    {
        const QueryCost candidate =
            costQuery(nodes, info, xmin, xmax, ymin, ymax,
                      info.spacingAtLevel(level));

        choice.chosenLevel = level;
        choice.resolution = info.spacingAtLevel(level);
        choice.cost = candidate;

        if (candidate.points <= pointBudget)
            break;
    }

    return choice;
}

} // namespace copc

#endif // COPC_INDEX_HPP
