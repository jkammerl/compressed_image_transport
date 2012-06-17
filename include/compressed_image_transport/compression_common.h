#ifndef COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON
#define COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON

namespace compressed_image_transport
{

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, JPEG, PNG, DEPTH
};

// Compression configuration
struct compressionConfiguration
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

} //namespace compressed_image_transport

#endif
