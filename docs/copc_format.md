# The COPC file format — LAS, LAZ and the octree

> [index](copc_laz_lod_tutorial.md) · **1. Format** · [2. LOD mechanisms](copc_lod_mechanisms.md) · [3. PDAL](copc_pdal.md) · [4. VTK camera](copc_vtk_camera.md) · [5. Worked examples](copc_worked_examples.md)

What is physically in the file: the LAS container, the LAZ codec, the two
VLRs that turn a LAZ into a COPC, and how that compares with EPT and Potree.

---

## 1. LAS — the container

LAS is a simple binary format from ASPRS. Four parts:

```
┌─────────────────────────────────────┐  byte 0
│ Public header block (fixed fields)  │  version, point format, point count,
│                                     │  scale/offset, min/max X Y Z, offsets
├─────────────────────────────────────┤  byte 375 (LAS 1.4)
│ Variable Length Records (VLRs)      │  CRS, ExtraBytes descriptors, the LAZ
│   54-byte header + payload, each    │  codec config, COPC's octree info,
│   keyed by (user_id, record_id)     │  and *anything custom*
├─────────────────────────────────────┤  "offset to point data" in the header
│ Point data records (fixed stride)   │  N × point_record_length
├─────────────────────────────────────┤  "start of first EVLR" in the header
│ Extended VLRs (EVLRs)               │  same idea, 60-byte header, 64-bit
│                                     │  length — COPC's hierarchy lives here
└─────────────────────────────────────┘
```

### 1.1 The public header block

375 bytes in LAS 1.4, and every field is at a fixed offset — which is why a
reader can learn the extent and point count of a 500 GB file with one tiny range
request. The fields that matter here, read live out of `synthetic.copc.laz`:

| Offset | Size | Field | Value in that file |
|---:|---:|---|---|
| 0 | 4 | File signature | `LASF` |
| 24 | 1+1 | Version major, minor | `1`, `4` |
| 26 | 32 | System identifier | `PDAL` |
| 58 | 32 | Generating software | `PDAL 2.10.0 (3ab424)` |
| 94 | 2 | Header size | `375` |
| 96 | 4 | Offset to point data | `683` |
| 100 | 4 | Number of VLRs | `2` |
| **104** | 1 | **Point Data Record Format** | `134` = `0x86` |
| 105 | 2 | Point record length | `30` |
| 131 | 8×3 | X, Y, Z **scale** | `0.01, 0.01, 0.01` |
| 155 | 8×3 | X, Y, Z **offset** | `0.0, 0.0, 0.0` |
| 179 | 8×6 | Max X, Min X, Max Y, Min Y, Max Z, Min Z | `999.0, 0.0, …` |
| 235 | 8 | Start of first EVLR | `5181469` |
| 243 | 4 | Number of EVLRs | `1` |
| 247 | 8 | Number of point records | `1000000` |
| 255 | 8×15 | Number of points by return | |

Two of those rows deserve a second look.

**`134` is not a point format.** `134 = 0x86 = 0x80 | 6`. LASzip sets the **high
bit** of the PDRF byte to mark the file as compressed; the real format is the
low four bits, here 6. lazperf's source comments this as *"Martin screws with
the high bits of the format, so we mask down to the low four bits"*
(`vendor/lazperf/header.cpp`). A LAS reader that forgets to mask sees format 134
and gives up. It is the single most common "why won't this file open" bug.

**Scale and offset are how LAS stores coordinates.** X, Y and Z are `int32` in
the point record; the real coordinate is

```text
real = stored_int32 * scale + offset
```

For a file with `scale = 0.00025` and `offset = 6483522` (`Palac_Moszna.laz` in
`data/las_files/`), a stored `1 234 567` means `6483522 + 308.64 = 6483830.64`.
Three consequences:

* **Precision is quantized to the scale**, everywhere in the file. `scale=0.01`
  means centimetres, full stop — there is no more precision in the file to
  recover, no matter what your `double` prints.
* **The offset exists because `int32` runs out.** UTM easting 515 400 m at 1 mm
  scale is 515 400 000 — fine — but at 0.25 mm it would be 2 061 600 000, right
  at the `int32` ceiling of 2 147 483 647. Subtracting an offset near the data's
  centre buys back the range. PDAL picks both automatically
  (`Grid::scale()` targets "a little less than 2 billion" for half the range).
* **Getting it wrong silently destroys data.** `writers.las scale_x=1.0` on a
  survey-grade cloud rounds every coordinate to the nearest metre and reports no
  error.

### 1.2 Variable Length Records

A VLR is a fixed 54-byte header followed by an arbitrary payload:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Reserved |
| 2 | 16 | **User ID** — a namespace, e.g. `LASF_Projection`, `LASF_Spec`, `copc` |
| 18 | 2 | **Record ID** — meaning within that namespace |
| 20 | 2 | Record length after header (so a VLR payload is capped at 65 535 bytes) |
| 22 | 32 | Description (free text) |
| **54** | | payload begins |

An **EVLR** is the same idea with a 60-byte header whose length field is a
`uint64`, and it lives *after* the point data. That is the only place a >64 kB
record can go, and it is where COPC's hierarchy pages live.

Read live from `synthetic.copc.laz` at offset 375:

```text
reserved   0
user_id    "copc"
record_id  1
len after  160
desc       "COPC info VLR"
           -> payload occupies bytes 429..588
```

`(user_id, record_id)` is the whole extensibility story: anyone can claim a
user ID and put whatever they like in the payload, and every reader that does
not recognise the pair skips it by its length. **This is exactly how COPC was
able to exist without changing LAS at all.**

The pairs you will actually meet (constants from PDAL's
`io/private/las/Vlr.hpp`):

| User ID | Record ID | What it is |
|---|---:|---|
| `LASF_Projection` | 2112 | **OGC WKT** coordinate reference system — required by LAS 1.4 for PDRF ≥ 6, and required by COPC |
| `LASF_Projection` | 34735 | **GeoTIFF** `GeoKeyDirectoryTag` — the LAS 1.0–1.3 way to store a CRS |
| `LASF_Projection` | 34736 | GeoTIFF `GeoDoubleParamsTag` — `double` values the keys point at |
| `LASF_Projection` | 34737 | GeoTIFF `GeoAsciiParamsTag` — strings the keys point at |
| `LASF_Spec` | 0 | Classification lookup — names for the 256 classification codes |
| `LASF_Spec` | 3 | Text area description — free-form notes about the file |
| `LASF_Spec` | 4 | **ExtraBytes** — declares the per-point fields appended to each record |
| `LASF_Spec` | 7 | Superseded |
| `LASF_Spec` | 100–354 | Waveform packet descriptors (formats 4, 5, 9, 10) |
| `laszip encoded` | 22204 | **LAZ codec configuration** — chunk size, per-field compressor versions |
| `copc` | 1 | **COPC info** — root cube, spacing, hierarchy location (see §3) |
| `copc` | 1000 | **COPC hierarchy** page (an EVLR) |
| `PDAL` | 12 | PDAL metadata, as JSON |
| `PDAL` | 13 | The full PDAL pipeline that produced the file, as JSON |
| `LASF_Projection` | 4224 | WKT2 (newer, PDAL-written) |
| `PDAL` | 4225 | PROJJSON |

#### The CRS, and its three generations

Coordinate reference systems in LAS have been encoded three different ways, and
you will meet all three in the wild.

**Generation 1 — GeoTIFF keys (LAS 1.0–1.3).** The CRS is borrowed wholesale
from the TIFF world: a `GeoKeyDirectoryTag` (34735) holding an array of
`(key_id, location, count, value)` quadruples, where `location = 0` means the
value is inline, and otherwise it names the VLR (34736 for doubles, 34737 for
ASCII) that holds the real value.

Here is a real one, decoded out of `data/las_files/Palac_Moszna.laz`:

```text
$ pdal info --metadata data/las_files/Palac_Moszna.laz

vlr_0  LASF_Projection / 34735  "GeoKeyDirectoryTag (mandatory)"   48 bytes
vlr_1  LASF_Projection / 34737  "GeoASCIIParamsTag (optional)"     44 bytes
vlr_2  laszip encoded  / 22204  "by laszip of LAStools (130126)"   72 bytes

GeoKeyDirectory: version 1.1.0, 5 keys
  key  1024  (GTModelTypeGeoKey)       inline        value 1      = projected
  key  1026  (GTCitationGeoKey)        in VLR 34737  count 44
  key  2052  (GeogLinearUnitsGeoKey)   inline        value 9001   = metre
  key  2054  (GeogAngularUnitsGeoKey)  inline        value 9102   = degree
  key  3076  (ProjLinearUnitsGeoKey)   inline        value 9001   = metre

GeoASCIIParams: "WGS84, WGS84, ETRS89 / Poland CS2000 zone 6\0"
```

**And this file is the cautionary tale.** It says "projected" and it names the
projection *in a free-text citation string* — but it never sets
`ProjectedCSTypeGeoKey` (3072), which is the key that would carry the EPSG code.
So there is nothing machine-readable to resolve, and PDAL/GDAL fall back to:

```text
LOCAL_CS["WGS84, WGS84, ETRS89 / Poland CS2000 zone 6",
         UNIT["metre",1,AUTHORITY["EPSG","9001"]],
         AXIS["Easting",EAST], AXIS["Northing",NORTH]]
```

A `LOCAL_CS` is an *engineering* CRS — coordinates with no georeference at all.
The numbers are still in ETRS89 / Poland CS2000 zone 6 (EPSG:2177), but no
software can know that, so reprojection is impossible until a human supplies it:

```bash
pdal translate in.laz out.copc.laz --readers.las.override_srs="EPSG:2177"
# or, when the file simply has no CRS and you know what it is:
pdal translate in.laz out.copc.laz --writers.copc.a_srs="EPSG:2177"
```

**Generation 2 — OGC WKT (LAS 1.4).** One VLR, `LASF_Projection` / 2112, holding
a WKT string. **LAS 1.4 requires WKT for point formats 6–10, and forbids the
GeoTIFF keys for them** — bit 4 of the header's Global Encoding field says which
is in use. Since COPC only permits formats 6, 7 and 8, **every valid COPC file
carries its CRS as WKT.** Writing one is a single option:

```bash
pdal translate points.csv out.copc.laz --writers.copc.a_srs="EPSG:25832"
```

```text
vlr_1  LASF_Projection / 2112
srs    PROJCS["ETRS89 / UTM zone 32N",
         GEOGCS["ETRS89",DATUM["European_Terrestrial_Reference_System_1989",...
```

**Generation 3 — WKT2 and PROJJSON.** Newer, richer, and written by PDAL
alongside the WKT1 record (`LASF_Projection`/4224 and `PDAL`/4225). Readers pick
among them; `readers.copc`'s `srs_vlr_order` option lets you state a preference
(`wkt1`, `wkt2`, `projjson`).

The practical upshot for a 1.2 → COPC conversion: **the CRS has to be
translated, and it can be lost.** Check it afterwards, every time:

```bash
pdal info --metadata out.copc.laz | jq -r .metadata.srs.horizontal
```

If that prints `LOCAL_CS[...]` or nothing, the georeference did not survive.

#### ExtraBytes: adding your own per-point fields

LAS point formats are fixed, so anything the ASPRS did not think of —
`Amplitude`, `Reflectance`, `Deviation`, `NormalX`, a segmentation label, a
confidence score — goes in **extra bytes appended to every point record**, with
a `LASF_Spec` / 4 VLR describing them. The descriptor is a fixed **192-byte**
struct per field (`ExtraBytesSpec` in PDAL's `io/private/las/Utils.hpp`):

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Reserved |
| 2 | 1 | **Data type** code (see below); `0` = undocumented raw bytes |
| 3 | 1 | Options bitfield — which of no_data / min / max / scale / offset are present |
| 4 | 32 | **Name** — the dimension name, e.g. `Amplitude` |
| 36 | 4 | Reserved |
| 40 | 24 | `no_data` (3 values, legacy) |
| 64 | 24 | `min` |
| 88 | 24 | `max` |
| 112 | 24 | `scale` |
| 136 | 24 | `offset` |
| 160 | 32 | Description |
| **192** | | next field's descriptor |

Data type codes: `1` uchar · `2` char · `3` ushort · `4` short · `5` uint32 ·
`6` int32 · `7` uint64 · `8` int64 · `9` **float** · `10` double.

A worked example — two `float` fields written onto PDRF 6:

```bash
pdal translate points.csv out.copc.laz \
     --writers.copc.extra_dims="Amplitude=float,Reflectance=float" \
     --writers.copc.a_srs="EPSG:25832"
```

```text
$ pdal info --metadata out.copc.laz

pdrf 6   point_record_length 38   count 20000
                              ^^
                              30 (PDRF 6 base) + 2 x 4 (two floats)

vlr_0  LASF_Spec / 4     384 bytes = 2 x 192
         [0] data_type=9 (float)  name='Amplitude'
         [1] data_type=9 (float)  name='Reflectance'
vlr_1  LASF_Projection / 2112     (the WKT CRS)
vlr_2  laszip encoded / 22204     "lazperf variant"
```

and PDAL then exposes them as ordinary dimensions alongside the built-in ones:

```text
X, Y, Z, Intensity, ReturnNumber, NumberOfReturns, ScanDirectionFlag,
EdgeOfFlightLine, Classification, Synthetic, KeyPoint, Withheld, Overlap,
ScanAngleRank, UserData, PointSourceId, GpsTime, ScanChannel,
Amplitude, Reflectance
```

Two things to keep in mind:

* **Extra bytes are not free, and they are not free in COPC either.** Every
  extra byte is paid on *every* point in *every* node you fetch. Two floats on
  PDRF 6 is `38/30` = a **27 % larger** record before compression, and LAZ
  compresses arbitrary extra bytes far worse than it compresses the fields it
  understands — it has no predictor for them.
* **`data_type = 0` means "undocumented"** — raw bytes with a size but no
  interpretation. PDAL surfaces them as `ExtraBytes0`, `ExtraBytes1`, … Use a
  real type code unless you truly mean opaque bytes.

#### Other VLRs worth knowing

* **Classification lookup** (`LASF_Spec` / 0) — 256 entries of `(code, 15-char
  description)`, so a producer can document that "class 20" means "power line"
  in *their* scheme. Rarely written; when present it is the only record of what
  the non-standard classes mean.
* **Text area description** (`LASF_Spec` / 3) — free text about the file.
* **The laszip VLR** (`laszip encoded` / 22204) — not metadata at all but the
  codec's own configuration: compressor type, chunk size, and a per-field list
  of which compressor version encodes which item. Strip it and the point data is
  undecodable. Note in the examples above that LAStools writes
  `"by laszip of LAStools (130126)"` and lazperf writes `"lazperf variant"` —
  the same format, two implementations.
* **`PDAL` / 13** — the complete pipeline JSON that produced the file. Turn it on
  with `--writers.copc.pipeline=true` and the file documents its own provenance.

### 1.3 Point data records

A point record is a fixed-size struct, laid out back to back with no padding and
no separators. Which fields exist is chosen by the **point data record format**
(PDRF), 0–10:

| PDRF | Size | = base + | Adds | LAZ? | COPC? |
|---:|---:|---|---|:-:|:-:|
| 0 | 20 | — | the core: XYZ, intensity, returns, classification, angle, source ID | ✅ | ❌ |
| 1 | 28 | 0 + 8 | GPS time | ✅ | ❌ |
| 2 | 26 | 0 + 6 | RGB | ✅ | ❌ |
| 3 | 34 | 1 + 6 | GPS time + RGB | ✅ | ❌ |
| 4 | 57 | 1 + 29 | GPS time + **waveform packet** | ❌ | ❌ |
| 5 | 63 | 3 + 29 | GPS time + RGB + waveform | ❌ | ❌ |
| **6** | **30** | — | LAS 1.4 core: 8-bit classification, 4-bit returns, scanner channel, GPS time **mandatory** | ✅ | ✅ |
| **7** | **36** | 6 + 6 | RGB | ✅ | ✅ |
| **8** | **38** | 7 + 2 | RGB + **NIR** | ✅ | ✅ |
| 9 | 59 | 6 + 29 | waveform | ❌ | ❌ |
| 10 | 67 | 8 + 29 | RGB + NIR + waveform | ❌ | ❌ |

(Sizes are lazperf's `baseCount()`; it returns `0` for 4, 5, 9 and 10 because
**LASzip cannot compress the waveform formats at all** — so those can never be
LAZ, and therefore never COPC.)

**COPC permits only 6, 7 and 8.** Formats 0–5 are the LAS 1.2 era: a 5-bit
classification field with only 32 possible classes, return numbers capped at 5,
and GPS time optional. Formats 6+ fix all three. This is why converting a
PDRF-3 LAS to COPC silently produces a PDRF-7 file — PDAL maps the fields
across, and the Scan Angle Rank (`int8`, whole degrees) becomes a Scan Angle
(`int16`, 0.006° steps) in the process.

**PDRF 6 in full, byte by byte** — the 30 bytes that every COPC point starts
with:

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 4 | X | `int32`; real = `X * scale_x + offset_x` |
| 4 | 4 | Y | |
| 8 | 4 | Z | |
| 12 | 2 | Intensity | `uint16`, normalized to 16 bits by the producer |
| 14 | 1 | Return Number : 4 bits, Number of Returns : 4 bits | up to 15 returns |
| 15 | 1 | Classification flags : 4 (Synthetic, Key-point, Withheld, **Overlap**), Scanner Channel : 2, Scan Direction : 1, Edge of Flight Line : 1 | |
| 16 | 1 | **Classification** | full `uint8`, 0–255 |
| 17 | 1 | User Data | producer's to define |
| 18 | 2 | Scan Angle | `int16`, 0.006° per count → ±180° |
| 20 | 2 | Point Source ID | usually the flight line |
| 22 | 8 | **GPS Time** | `double`; mandatory in 6+ |
| **30** | | *(PDRF 7 appends R, G, B as `uint16`; PDRF 8 appends NIR)* | |

Two fields carry more meaning than their size suggests:

**Classification** is the ASPRS standard list, and it is worth memorising the
first dozen because every filter you write uses them:

| Code | Meaning | | Code | Meaning |
|---:|---|---|---:|---|
| 0 | Never classified | | 7 | Low point (noise) |
| 1 | Unassigned | | 9 | Water |
| **2** | **Ground** | | 10 | Rail |
| 3 | Low vegetation | | 11 | Road surface |
| 4 | Medium vegetation | | 13–15 | Wires: guard, conductor, tower |
| **5** | **High vegetation** | | 17 | Bridge deck |
| **6** | **Building** | | 18 | High noise |

```bash
# ground only, from one region, at 2 m spacing
pdal translate big.copc.laz ground.las \
     --readers.copc.bounds="([515370,515400],[4918340,4918370])" \
     --readers.copc.resolution=2 \
     --filters.range.limits="Classification[2:2]"
```

Worth repeating, because it is the most common misconception about COPC: the
`bounds` and `resolution` options prune the octree traversal and genuinely
reduce I/O; **`filters.range` runs after decompression and saves no bytes at
all.** See [§4](copc_pdal.md).

**Return number / number of returns** is what makes a lidar pulse multi-valued:
one emitted pulse can echo off a leaf, a branch and the ground, producing 3
records with `NumberOfReturns = 3` and `ReturnNumber` 1, 2, 3. `ReturnNumber ==
NumberOfReturns` is the classic "last return ≈ ground" heuristic, and it is why
a raw cloud has far more points than it has *surface*.

**A full worked decode.** Given a header with `scale = (0.01, 0.01, 0.01)`,
`offset = (500000, 5400000, 0)`, and these 30 bytes:

```text
offset  bytes                       field              decoded
------  --------------------------  -----------------  ---------------------------
  0     87 D6 12 00                 X   = 1234567      500000 + 12345.67 = 512345.67
  4     15 CD 5B 07                 Y   = 123456789    5400000 + 1234567.89 = 6634567.89
  8     E8 03 00 00                 Z   = 1000         0 + 10.00 = 10.00
 12     D0 07                       Intensity = 2000
 14     31                          0x31 = 0011 0001   return 1 of 3
 15     00                                             no flags, channel 0
 16     05                          Classification = 5 high vegetation
 17     00                          User Data = 0
 18     00 00                       Scan Angle = 0     0.000 degrees
 20     01 00                       Point Source ID = 1
 22     ... 8 bytes ...             GPS Time
```

Note `X = 1234567` is a *count of centimetres*, not a coordinate. Everything
about LAS's compactness — and about LAZ's compression ratio — follows from
storing small integers rather than doubles.

Finally, two properties of the record layout that shape everything downstream:

1. **Fixed stride means point *i* is at a computable offset:**
   `offset_to_point_data + i * point_record_length`. Random access by index is
   free in LAS.
2. **File order carries no spatial meaning.** Point *i* and point *i+1* can be
   kilometres apart — they are usually in acquisition order, which follows the
   flight line, not the ground. **Random access by index is useless for a
   viewer**, which always wants "the points in *this box*". Fixing that is
   exactly what COPC does (§3).

## 2. LAZ — the compression

LAZ (LASzip) is lossless compression of the LAS point records. It is not gzip
over the file; it is a format-aware arithmetic coder that predicts each field
from the previous point (delta-encode X, Y, Z; the classification rarely
changes; GPS time is monotonic). Typical 5–10× reduction.

The codec is configured by the `laszip encoded` / 22204 VLR, and you can read
its settings straight out of any LAZ file. `data/las_files/Palac_Moszna.laz`
(LAS 1.2, PDRF 3, written by LAStools) says:

```text
compressor   2  (pointwise_chunked)
chunk_size   50000
version      2.1.0
items        POINT10  size 20  v2      <- the PDRF-0 core, one compressor
             GPSTIME11 size  8  v2     <- GPS time, another
             RGB12    size  6  v2      <- colour, another
```

Note that LAZ compresses the record **field group by field group**, not as an
opaque blob: each item has its own predictor and its own version. That is also
why arbitrary ExtraBytes compress poorly — they arrive as a `BYTE`/`BYTE14`
item with no model of what the numbers mean.

Crucially LAZ is **chunked**: points are grouped (`chunk_size` above, 50 000 by
default) and each chunk is independently decompressible. A chunk table at the
end of the file lists each chunk's point count and byte size.

```
LAZ file:
 [hdr][chunk 0][chunk 1][chunk 2] ... [chunk N][chunk table]
        50k pts  50k pts  50k pts
```

So random access exists — but only by *chunk index*, and chunk index means
nothing spatially. Worse, file order is acquisition order, so chunk 7 is "the
eighth 50 000 points the sensor recorded", which is a smear along a flight
line, not a place. To find every point in a 100 m box you still have to
decompress the entire file. **This is exactly the gap COPC fills**, and it fills
it by changing `chunk_size` to *variable* and letting the octree place the
boundaries — see §3 below.

## 3. COPC — LAZ chunks reorganized into an octree

COPC (Cloud Optimized Point Cloud) changes no bytes of the LAZ codec. It makes
two additions:

1. **Each LAZ chunk is exactly one octree node.** The points are reordered
   before writing so that this holds.
2. **Two VLRs describe the octree:** a `copc info` VLR (root cube centre and
   half-size, root node spacing, offset to the root hierarchy page) and one or
   more `copc hierarchy` VLRs, each a flat array of entries:

```
struct Entry {          // 32 bytes
    VoxelKey key;       // level, x, y, z   (4 × int32)
    uint64   offset;    // byte offset of this node's LAZ chunk
    int32    byteSize;  // compressed size
    int32    pointCount;// > 0 = node, 0 = empty, -1 = this key is a child page
};
```

That `-1` case is what makes the index itself lazy: the hierarchy is paged, so a
client does not download the index for a region it never looks at.

A reader therefore needs: **one range request for the header + info VLR, one for
the root hierarchy page, then one range request per node it decides to draw.**
Over HTTP that is plain `Range:` headers — no server software, just a static
file on S3. That is the entire point of "cloud optimized".

### Where are the chunks actually specified?

This is the question the diagram above hides: **who decides where one chunk ends
and the next begins?** The answer is different for plain LAZ and for COPC, and
the difference is the whole trick.

**In plain LAZ, the chunk size is a number in a VLR.** The `laszip encoded` /
22204 VLR is the codec's configuration record, and one of its fields is
`chunk_size`. Decoded out of `data/las_files/Palac_Moszna.laz`:

```text
compressor      2  (pointwise_chunked)
chunk_size      50000               <-- here it is: cut every 50 000 points
num_items       3      version 2.1.0
  item POINT10      size 20  version 2
  item GPSTIME11    size  8  version 2
  item RGB12        size  6  version 2
```

The writer counts to 50 000, calls `done()` on the arithmetic coder, records the
chunk's compressed size, and starts a new one. Chunk boundaries fall wherever
the point *counter* says — which, since file order is acquisition order, is
somewhere arbitrary along a flight line.

**In COPC, nothing specifies a chunk size, because the octree already did.**
The same VLR from `synthetic.copc.laz`:

```text
compressor      3  (layered_chunked)
chunk_size      0xFFFFFFFF          <-- the sentinel for "VARIABLE"
num_items       1      version 3.4.3
  item POINT14      size 30  version 3
```

`0xFFFFFFFF` means *there is no fixed size; every chunk's length is recorded
individually*. So the boundaries come from somewhere else entirely — and that
somewhere is one function in PDAL's writer. `Processor::writeCompressed()` is
called **once per octree node**, and it creates a brand-new compressor for that
node, compresses exactly that node's points, and closes it:

```cpp
void Processor::writeCompressed(VoxelKey k, PointViewPtr v)
{
    lazperf::writer::chunk_compressor compressor(b.pointFormatId, b.numExtraBytes);
    v->sort(Dimension::Id::GpsTime);
    for (PointId idx = 0; idx < v->size(); ++idx)
        compressor.compress(...);                   // this node's points, all of them
    std::vector<unsigned char> chunk = compressor.done();   // <-- chunk boundary, here
    uint64_t location = m_manager.newChunk(k, chunk.size(), v->size());
    ...seek to `location`, write `chunk`...
}
```

**That `done()` is the chunk boundary.** One node in, one chunk out, whatever
size it happens to be — which is why the real node list in
[the LOD-mechanisms chapter](copc_lod_mechanisms.md) ranges from 318 points
(1 733 bytes) to 126 731 points (585 499 bytes) in the same file. Nobody chose
those numbers; the octree did.

The other half of the answer is how a reader ever finds them again.
`Output::newChunk()` assigns the offset and records it in **two places at once**:

```cpp
uint64_t Output::newChunk(const VoxelKey& key, int32_t size, int32_t count)
{
    if (count == 0)                          // an empty node still gets an entry
    {
        m_hierarchy[key] = { 0, 0, 0 };
        return 0;
    }
    uint64_t chunkStart = m_pointPos;
    m_pointPos += size;
    m_chunkTable.push_back({ (uint64_t)count, (uint64_t)size });   // 1. the LAZ chunk table
    m_hierarchy[key]  = { chunkStart, size, count };               // 2. the COPC hierarchy
    return chunkStart;
}
```

So:

| | Plain LAZ | COPC |
|---|---|---|
| Where the boundary is decided | `chunk_size` field in the laszip VLR | one `chunk_compressor` per octree node |
| Typical boundary | every 50 000 points | every node, 318 … 126 731 points |
| How a reader finds chunk *i* | the **chunk table** at the end of the file | the **hierarchy VLR**, keyed by `(level, x, y, z)` |
| What the key means | a sequence number | a box in space at a level of detail |

**Both records are written from the same call**, which is why the LAZ chunk
table and the COPC hierarchy can never disagree: a plain LAZ reader walks the
chunk table front to back and sees a normal cloud, while a COPC reader looks up
a VoxelKey and jumps straight to that node's bytes. Same bytes, two indexes over
them — and only one of them knows anything about space.

### Octree first, chunks second — how the two are made to line up

Three things trip people up here, so take them one at a time.

**Misconception 1: "COPC chunks have a fixed point count."** They do not. That
is true of plain LAZ (`chunk_size = 50000`) and it is exactly what COPC throws
away by setting `chunk_size = 0xFFFFFFFF`. Here is the real chunk layout of
`synthetic.copc.laz` (1 M points, written by PDAL), read out of its hierarchy:

```text
 #  key         offset     bytes   points   gap
 0  2-0-0-0        691    244017    48013
 1  2-1-0-0     244708    243574    48072      0
 2  2-0-1-0     488282    245478    48126      0
 3  2-1-1-0     733760    245051    48213      0
 4  2-0-2-0     978811    243156    47975      0
 …                                             (16 level-2 chunks in all)
15  2-3-3-0    3662440    243520    47924      0
16  1-0-0-0    3905960    279525    50771      0
17  1-1-0-0    4185485    281084    51016      0
18  1-0-1-0    4466569    280542    50996      0
19  1-1-1-0    4747111    283022    51389      0
20  0-0-0-0    5030133    151248    27279      0
                        ─────────  ───────
                          5180690  1000000
```

No two chunks have the same point count. They are *similar* — 47 924 to 51 389 —
because the sampling grid gives every node about the same number of points, but
nothing enforces it. And on a real, unevenly dense cloud they are not even
similar: lone-star's chunks run from **318 points to 126 731 points**.

**Misconception 2: "we make LAZ chunks, then fit an octree over them."** There
is no fitting step, because it would be impossible — a fixed-size chunk cut from
acquisition order contains points scattered across the whole survey. Running
`laszip` on a LAS gives you 50 000-point chunks in flight-line order and **no
octree at all**; nothing can be retrofitted onto that without rewriting every
point. `LAS → LAZ` and `LAS → COPC` are different operations.

**The actual order is: bin → sample → emit.** The octree is built first, and a
chunk is the *output* of a node, not a container a node has to fit into. Every
gap in the table above is `0`, and the chunks appear in bottom-up order —
sixteen level-2 nodes, then four level-1 nodes, then the root last — because
each chunk is written the moment its node is finalized, and a parent cannot be
finalized until all its children are.

Here is `writers.copc` end to end, with the source file for each phase:

```text
┌─ CopcWriter::write()  ────────────────────────────────────────────────────┐
│                                                                            │
│ 1. STATS + ROOT CUBE                                     Grid.cpp          │
│    one pass over the points for min/max, then                              │
│      side = max(xside, yside, zside)                                       │
│      spacing = side / 147                                                  │
│                                                                            │
│ 2. PICK THE DEEPEST LEVEL                                Grid::calcLevel() │
│    halve the point count per level until a node would hold                 │
│    ≤ MaxPointsPerNode (100 000):                                           │
│      while (mp > 100000/1e6) { mp /= 2 per long axis; side /= 2; level++ } │
│    1 M points -> level 2.   gridSize = 2^level = 4 cells per side.         │
│                                                                            │
│ 3. BIN EVERY POINT INTO A LEAF CELL              CellManager, Grid::key()  │
│    for (PointRef p : *v)                                                   │
│        mgr.get(grid.key(x,y,z))->appendPoint(...)                          │
│    THIS is the reorder. After it, points are grouped by leaf cell —        │
│    i.e. by position — and acquisition order is gone.                       │
│                                                                            │
│ 3b. RESCUE OVERFULL CELLS                                Reprocessor.cpp   │
│    any cell that still holds ≥ 100 000 points is re-binned n levels        │
│    deeper, n = ceil(log2(size/100000)/2).  Dense patches get a locally     │
│    deeper tree; the rest of the file is untouched.                         │
│                                                                            │
│ 4. BOTTOM-UP SAMPLE AND EMIT               BuPyramid.cpp, Processor.cpp    │
│    a node is processed once all 8 children are done:                       │
│      sample()          promote one point per grid cell into the parent     │
│      writeCompressed() one chunk_compressor per child, done() -> a chunk   │
│      newChunk()        assign its offset; record it in BOTH the LAZ        │
│                        chunk table and the COPC hierarchy                  │
│    the root is written last, by its own processor.                         │
│                                                                            │
│ 5. FINALIZE                                              Output.cpp        │
│    chunk table, hierarchy pages as EVLRs, copc info VLR, LAS header        │
└────────────────────────────────────────────────────────────────────────────┘
```

Read phase 3 again, because it is the answer to the question. **The chunk
boundary is never chosen; the cell boundary is.** `grid.key(x, y, z)` decides
which cell a point belongs to, purely from its coordinates:

```cpp
VoxelKey Grid::key(double x, double y, double z)
{
    int xi = (int)std::floor((x - m_bounds.minx) / m_xsize);
    int yi = (int)std::floor((y - m_bounds.miny) / m_ysize);
    int zi = (int)std::floor((z - m_bounds.minz) / m_zsize);
    ...
    return VoxelKey(xi, yi, zi, m_maxLevel);
}
```

Every point lands in exactly one cell; each cell becomes one node; each node
becomes one chunk. Spatial fit is not achieved, it is **definitional**.

Two practical consequences fall out of this design:

* **`writers.copc` is not streamable, and holds the whole cloud in memory.**
  `CellManager` is an `unordered_map<VoxelKey, PointViewPtr>` with no spill to
  disk, and phase 2 needs the total point count before it can choose a level, so
  the writer cannot start until the last point has arrived. In PDAL's class
  hierarchy `CopcReader` derives from `Streamable` and `CopcWriter` does not:
  **you can stream out of a COPC, never into one.** For clouds too large for
  RAM, tile the input and write one COPC per tile, or use Entwine, which is
  out-of-core by design.
* **File order is now finest-first.** A plain LAZ reader walking
  `synthetic.copc.laz` front to back gets the sixteen level-2 nodes first and
  the coarse overview last. Perfectly legal — chunk order carries no meaning in
  LAZ — but it is why the "oddly ordered point cloud" caveat keeps appearing:
  the order is meaningful now, just not to a reader that ignores the index.

### The VoxelKey and the cube

```
level 0:                ┌───────────────┐
  key (0,0,0,0)         │               │   one cube covering the whole
  spacing = s           │       •       │   dataset (cubic, not the bbox)
                        │               │
                        └───────────────┘

level 1:                ┌───────┬───────┐
  keys (1,x,y,z)        │ •  •  │ •  •  │   8 children, each half the edge
  x,y,z ∈ {0,1}         ├───────┼───────┤   spacing = s/2
  spacing = s/2         │ •  •  │ •  •  │
                        └───────┴───────┘

level 2:                ┌───┬───┬───┬───┐
  spacing = s/4         │•·•│•·•│•·•│•·•│   64 children
                        ├───┼───┼───┼───┤   spacing = s/4
                        │•·•│•·•│•·•│•·•│
                        ├───┼───┼───┼───┤
                        │•·•│•·•│•·•│•·•│
                        ├───┼───┼───┼───┤
                        │•·•│•·•│•·•│•·•│
                        └───┴───┴───┴───┘
```

The bounds of a node are pure arithmetic from the key and the root cube — no
lookup needed:

```cpp
double size = rootSize / (1 << key.level);
minx = rootMinX + key.x * size;   maxx = minx + size;
// same for y, z
```

### The one idea that makes LOD work: nodes are *additive*, not a partition

This is the part people get wrong. In a classic spatial index (an R-tree, a
kd-tree), all the data lives in the leaves and the internal nodes are just
routing. In COPC/EPT/Potree, **every node holds real points**, and a node's
points are a *thinned sample* of everything below it, at that node's spacing.

```
level 0  ·     ·     ·     ·        ~65k pts, 1 pt every 2.0 m
level 1  · ·  · ·  · ·  · ·         ~65k pts/node, 1 pt every 1.0 m
level 2  ·············               ~65k pts/node, 1 pt every 0.5 m
level 3  ···············             ~65k pts/node, 1 pt every 0.25 m

what you draw = level 0  ∪  level 1  ∪  level 2  ∪  ...
```

Consequences you can rely on:

* Every node holds roughly the **same number of points** (the chunk budget).
  Cost per node is constant; cost is proportional to *node count*, which is what
  makes budgeting a frame possible.
* Refinement is **incremental**. Descending one level does not invalidate what
  you already drew — you *add* to it. A viewer can render level 0 instantly and
  keep appending, and nothing ever flickers or has to be re-fetched.
* **Spacing halves per level**: `spacing(d) = rootSpacing / 2^d`. That is the
  bridge from "how many pixels is this on screen" to "which level do I need".

The root spacing is stored in the COPC info VLR, and the ratio
`rootCubeSize / rootSpacing` is the writer's one real design parameter: PDAL
uses 147, the writer of `lone-star.copc.laz` used 128. See
[the LOD-mechanisms chapter](copc_lod_mechanisms.md) for both, measured.
### The complete COPC file, byte by byte

§3's two VLRs and [the LOD-mechanisms chapter](copc_lod_mechanisms.md)'s four mechanisms are the parts
you *use*. For completeness, here is everything the specification actually
pins down, because a few of the constraints bite in practice.

```text
offset      0   LAS 1.4 public header block                        375 bytes
offset    375   VLR header, user_id "copc", record_id 1             54 bytes  <- MUST be first
offset    429   copc info VLR payload                              160 bytes  <- fixed offset
offset    589   VLR header, user_id "laszip encoded", record_id 22204
                laszip VLR payload (LAZ codec configuration)
                VLR header, "LASF_Projection", record_id 2112
                OGC WKT coordinate reference system                           <- required
                any other VLRs: ExtraBytes (4), PDAL pipeline, user VLRs
                ─────────────────────────────────────────────
                [chunk 0][chunk 1][chunk 2] ... [chunk N]                     <- one chunk per node
                LAZ chunk table (its offset is written just before chunk 0)
evlr_offset     EVLR header, user_id "copc", record_id 1000         60 bytes
                root hierarchy page: 32 bytes x entries
                further hierarchy pages, further EVLRs
```

The `copc info` VLR payload, all 160 bytes of it (this is the whole struct —
`lazperf/vlr.hpp`, `copc_info_vlr`):

| Field | Type | Bytes | Meaning |
|---|---|---:|---|
| `center_x`, `center_y`, `center_z` | `double` ×3 | 24 | centre of the **cubic** root node |
| `halfsize` | `double` | 8 | half the root cube edge; the cube is cubic even when the data's bbox is not |
| `spacing` | `double` | 8 | root node spacing — **the entire LOD ladder** |
| `root_hier_offset` | `uint64` | 8 | byte offset of the root hierarchy page |
| `root_hier_size` | `uint64` | 8 | its size; `/32` is the number of entries |
| `gpstime_minimum` | `double` | 8 | whole-file GPS time range, or 0 if the writer did not fill it |
| `gpstime_maximum` | `double` | 8 | |
| `reserved` | `uint64` ×11 | 88 | must be zero |
| | | **160** | |

Constraints COPC adds on top of LAS/LAZ, each of which has bitten somebody:

* **LAS 1.4 only, and point data record format 6, 7 or 8.** These are the
  formats where GPS time is mandatory and the legacy 8-bit classification is
  gone. PDRF 0–5 files are *not* valid COPC; `writers.copc` converts silently
  on the way in, which is why a PDRF-3 LAS turns into a PDRF-7 COPC without
  telling you.
* **CRS must be OGC WKT** (VLR `LASF_Projection` / 2112). The GeoTIFF key VLRs
  that LAS 1.2 files carry are not permitted, so a 1.2 → COPC conversion has to
  translate the CRS, and can lose it entirely if the GeoTIFF keys were odd.
* **The info VLR is at a fixed offset.** `375` for the VLR header, `429` for the
  payload, always. That is the only reason `copc_hierarchy_inspect` can be
  dependency-free: there is no VLR scan, you seek to 429 and read 160 bytes.
  It is also why a client knows everything structural about a file after a
  **single 589-byte range request**.
* **The hierarchy is in EVLRs, at the *end* of the file.** It has to be: a
  writer only knows every chunk's offset once every chunk is written. This is
  also why a truncated download of a COPC file is useless — you lose the index,
  not the tail of the data.
* **Chunks are variable-sized and the chunk table is mandatory.** Plain LAZ
  allows a fixed 50 000-point chunking with no table; COPC does not.
* **`reserved` must be zero**, and `pointCount` in a hierarchy entry is signed
  precisely so `-1` can mean "child page".
* **There is no magic number.** A `.copc.laz` is a `.laz`; the only thing that
  makes it COPC is a valid info VLR sitting at offset 375. The `.copc.laz`
  double extension is convention, not spec.

**Checking whether a file is actually COPC** (rather than a LAZ someone renamed):

```bash
pdal info --metadata big.copc.laz | jq .metadata.copc_info
```

```json
{
  "center_x": 515388.9821, "center_y": 4918360.744, "center_z": 2343.276125,
  "halfsize": 20.379875,   "spacing": 0.3184355469,
  "root_hier_offset": 2704713, "root_hier_size": 480,
  "gpstime_minimum": 0, "gpstime_maximum": 0
}
```

No `copc_info` block → it is plain LAZ and every read will be a full-file read.
`root_hier_size / 32` is the node count of the root page (here `480/32 = 15`).
`./copc_hierarchy_inspect` prints the same thing plus the tree.

GPS time, incidentally, is the one dimension COPC indexes *outside* the octree,
and only at file granularity. `lone-star` above reports `0 / 0` — legal, and
common. `autzen_clip_native.copc.laz` reports `374103204.2 / 374103810`, about
ten minutes of flight, which is enough to skip a whole file in a catalogue of
thousands, but not enough to skip a node inside one.

### What COPC deliberately does not do

Knowing the holes saves you from designing around a feature that is not there:

| Not in COPC | Consequence | What to do instead |
|---|---|---|
| **Append / update** | Files are immutable. Adding a single point means rewriting the whole file and its index. | Tile into many COPC files; rewrite one tile |
| **Attribute index** | `bounds` and `resolution` are the *only* prunings the index can answer. A `Classification == 2` filter reads every byte first. | `filters.range` downstream — it saves RAM, never I/O. Or pre-split by class into separate files |
| **Server-side query** | No API, no tile server, no protocol. | It is a static file behind `Range:`. That is the feature, not the gap |
| **Per-node time range** | `gpstime_minimum/maximum` is file-level only | Tile by time as well as space |
| **Progressive attributes** | A node carries all its dimensions or none. You cannot stream XYZ now and RGB later. | Nothing — budget for the full record width |
| **Skirts / node overlap** | A neighbour query near a node edge needs the neighbouring node | Expand your query box by the node width before cropping |
| **Multi-file catalogues** | One file, one octree, one CRS | `pdal tindex` builds a spatial index *over* COPC files |
| **Anything but points** | No meshes, no normals-as-LOD, no textures | — |

The single most common surprise on that list is the attribute index. **Reading
"only the ground points" from a COPC costs exactly as many bytes as reading
everything in that region.** The octree prunes in space and in depth, and in
nothing else.

---

## 5. Potree, EPT, COPC — same idea, three packagings

All three descend from Entwine's EPT. They differ only in how the octree is
stored on disk.

| | Layout | Index | Best at |
|---|---|---|---|
| **EPT** | one file per node (`ept-data/0-0-0-0.laz`) + JSON hierarchy | JSON files | Simple, but millions of tiny files |
| **Potree 2.0** | 3 files: `octree.bin`, `hierarchy.bin`, `metadata.json` | binary hierarchy, paged | Browser streaming; native viewer format |
| **COPC** | **one** `.copc.laz`, valid LAZ for any LAS reader | VLRs inside the file | Archival + cloud; nothing to unpack |

The killer property of COPC is **backward compatibility**: `big.copc.laz` opens
in any LAS/LAZ reader on earth, which sees a normal (oddly ordered) point cloud
and ignores the two VLRs. Potree 2.0's `octree.bin` opens in nothing.

So, in practice:

* **COPC** = storage and interchange. Serve it from S3, done.
* **Potree** = a *viewer* (and its converter). Potree 2.0 can load COPC directly
  now, so you often skip PotreeConverter entirely.
* **PDAL** = the tool that reads and writes all of them, plus 30 other formats.

**VTK's octrees are not in this family at all.** `vtkOctreePointLocator`,
`vtkIncrementalOctreePointLocator` ([docs/octree.md](octree.md)),
`vtkHierarchicalBinningFilter` ([docs/hierarchical_binning_filter.md](hierarchical_binning_filter.md))
are *in-memory* structures built from a `vtkPolyData` you already loaded, for
nearest-neighbour and radius queries. They index what is in RAM; COPC indexes
what is on disk. Building a `vtkOctreePointLocator` does not give you COPC, and
reading COPC does not give you a locator.

