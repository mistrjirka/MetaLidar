#pragma once

typedef struct Time {
  uint32 sec;
  uint32 nsec;
} Time;

// ROS standard header file
typedef struct Header {
  uint32 seq;         // sequence ID
  Time stamp;         // timestamp in seconds
  FString frame_id; // frame ID in which the data is observed
} Header;


// Data structure for a single field in PointCloud2
typedef struct PointField {
  FString name; // Name of the field (e.g., x, y, z, intensity)
  uint32 offset;  // Offset from the start of the point struct
  uint8 datatype; // Data type of the elements (e.g., FLOAT32, UINT8)
  uint32 count; // Number of elements in the field (e.g., 1 for x, y, z; 3 for RGB)

  // Constants for data types
  static const uint8 INT8 = 1;
  static const uint8 UINT8 = 2;
  static const uint8 INT16 = 3;
  static const uint8 UINT16 = 4;
  static const uint8 INT32 = 5;
  static const uint8 UINT32 = 6;
  static const uint8 FLOAT32 = 7;
  static const uint8 FLOAT64 = 8;

  PointField(FString name, uint32 offset, uint8 datatype, uint32 count)
    : name(name), offset(offset), datatype(datatype), count(count) {}
} PointField;

// The PointCloud2 message structure
typedef struct PointCloud2 {
  Header header;   // Standard ROS message header
  uint32 height; // Height of the point cloud dataset
  uint32 width;  // Width of the point cloud dataset
  uint32 numOfFields;
  TArray<PointField>
      fields; // Describes the channels and their layout in the binary data blob
  bool is_bigendian;   // Is the data big-endian
  uint32 point_step; // Length of a point in bytes
  uint32 row_step;   // Length of a row in bytes
  TArray<uint8>
      data;      // Actual point cloud data, size is (row_step*height)
  bool is_dense; // True if there are no invalid points (NaN or Inf)
} PointCloud2;

typedef struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
} PointXYZI;

typedef struct
{
  pthread_mutex_t mutex;
  uint32 seq;
  size_t packet_size;
  uint8 data[];
} MemoryPacket;

typedef struct RQuaternion {
  double x;
  double y;
  double z;
  double w;
} RQuaternion;

typedef struct RVector3 {
  double x;
  double y;
  double z;
} RVector3;