#include "compressed_image_transport/compressed_publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <highgui.h>
#include <boost/make_shared.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace compressed_image_transport
{

void CompressedPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                        const image_transport::SubscriberStatusCallback &user_connect_cb,
                                        const image_transport::SubscriberStatusCallback &user_disconnect_cb,
                                        const ros::VoidPtr &tracked_object, bool latch)
{
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&CompressedPublisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void CompressedPublisher::configCb(Config& config, uint32_t level)
{
  config_ = config;
}

void CompressedPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  // Compressed image message
  sensor_msgs::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = config_.format;

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  if (config_.format == "jpeg")
  {
    params[0] = CV_IMWRITE_JPEG_QUALITY;
    params[1] = config_.jpeg_quality;

    // OpenCV-ros bridge
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(message);

    // Check input format
    if (((cv_ptr->image.channels() == 1) || (cv_ptr->image.channels() == 3))
        &&    ((cv_ptr->image.depth() == CV_8U)
            || (cv_ptr->image.depth() == CV_8S)
            || (cv_ptr->image.depth() == CV_16U)
            || (cv_ptr->image.depth() == CV_16S)))
    {

      // Compress image
      if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
      {

        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
                       (float)compressed.data.size();
        ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());

        // Publish message
        publish_fn(compressed);
      }
      else
      {
        ROS_ERROR("cv::imencode (jpeg) failed on input image");
      }
    }
    else
      ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit, 1/3-channel (with BGR channel order) images (input format is: %s)", message.encoding.c_str());

  }
  else if (config_.format == "png")
  {
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = config_.png_level;

    // OpenCV-ros bridge
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message);

    // Check input format
    if (((cv_ptr->image.channels() == 1) || (cv_ptr->image.channels() == 3))
        &&    ((cv_ptr->image.depth() == CV_8U)
            || (cv_ptr->image.depth() == CV_8S)
            || (cv_ptr->image.depth() == CV_16U)
            || (cv_ptr->image.depth() == CV_16S)))
    {
      // Compress image
      if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
      {

        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
            / (float)compressed.data.size();
        ROS_DEBUG(
            "Compressed Image Transport - Codec: png, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());

        // Publish message
        publish_fn(compressed);
      }
      else
      {
        ROS_ERROR("cv::imencode (png) failed on input image");
      }
    } else
      ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit, 1/3-channel (with BGR channel order) images (input format is: %s)", message.encoding.c_str());
  }
  else if (config_.format == "depth")
  {
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = config_.png_level;

    float depthZ0 = config_.depth_quantization;
    float depthMax = config_.depth_max;

    // OpenCV-ROS bridge
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message);

    // Check input format
    if ((cv_ptr->image.depth() == CV_32F) && (cv_ptr->image.channels() == 1))
    {
      const Mat& depthImg = cv_ptr->image;
      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      // Allocate matrix for inverse depth (disparity) coding
      Mat invDepthImg(rows, cols, CV_16UC1);

      // Inverse depth quantization parameters
      float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
      float depthQuantB = 1.0f - depthQuantA / depthMax;

      // Matrix iterators
      MatConstIterator_<float> itDepthImg     = depthImg.begin<float>(),
                                itDepthImg_end = depthImg.end<float>();
      MatIterator_<unsigned short> itInvDepthImg     = invDepthImg.begin<unsigned short>(),
                                     itInvDepthImg_end = invDepthImg.end<unsigned short>();

      // Quantization
      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
      {
        // check for NaN & max depth
        if (*itDepthImg < depthMax)
        {
          *itInvDepthImg = depthQuantA / *itDepthImg + depthQuantB;
        }
        else
        {
          *itInvDepthImg = 0;
        }
      }

      // Add coding parameters to data stream
      compressed.data.resize(sizeof(float) * 2);
      memcpy(&compressed.data[0], &depthQuantA, sizeof(float));
      memcpy(&compressed.data[sizeof(float)], &depthQuantB, sizeof(float));

      // Compress quantized disparity image
      std::vector<uint8_t> compressedImage;
      if (cv::imencode(".png", invDepthImg, compressedImage, params))
      {
        compressed.data.insert(compressed.data.end(), compressedImage.begin(), compressedImage.end());

        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
                        (float)compressed.data.size();
        ROS_DEBUG("Compressed Image Transport - Codec: depth, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());

        // Publish message
        publish_fn(compressed);
      }
      else
      {
        ROS_ERROR("cv::imencode (png) failed on input image");
      }
    } else
      ROS_ERROR("Compressed Image Transport - Depth compression requires single-channel 32bit-floating point images (input format is: %s).", message.encoding.c_str());
  }
  else
  {
    ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg', 'png' and 'depth'", config_.format.c_str());
    return;
  }

}

} //namespace compressed_image_transport
