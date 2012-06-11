#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <limits>
#include <vector>

#define ELEM(type,start,step,size,xpos,ypos) *((type*)(start+step*(ypos)+(xpos)*size))

using namespace cv;

namespace compressed_image_transport
{

void CompressedSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)

{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->header = message->header;

  if (message->format == "depth")
  {
    // Depth decoding

    if (message->data.size() > 8)
    {
      float depthQuantA, depthQuantB;

      // Decode quantization parameters in header
      memcpy(&depthQuantA, &message->data[0], sizeof(float));
      memcpy(&depthQuantB, &message->data[sizeof(float)], sizeof(float));

      // Decode compressed image
      const vector<uint8_t> pngData(message->data.begin() + 2 * (sizeof(float)), message->data.end());
      cv::Mat decompressed = cv::imdecode(pngData, CV_LOAD_IMAGE_UNCHANGED);

      // Allocate floating point image
      int rows = decompressed.rows;
      int cols = decompressed.cols;
      cv_ptr->image = Mat(rows, cols, CV_32FC1);

      // Depth depth conversion
      MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(), itDepthImg_end = cv_ptr->image.end<float>();
      MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(), itInvDepthImg_end =
                                            decompressed.end<unsigned short>();

      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
      {
        // check for NaN & max depth
        if (*itInvDepthImg)
        {
          *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
        }
        else
        {
          *itDepthImg = std::numeric_limits<float>::quiet_NaN();
        }
      }

    }

    // Assign encoding
    cv_ptr->encoding = "32FC1";

  }
  else
  {

    // Decode image
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), CV_LOAD_IMAGE_UNCHANGED);

    // Assign encoding to ROS message
    switch (cv_ptr->image.type() & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK))
    {
      case CV_8UC1:
        cv_ptr->encoding = "8UC1";
        break;
      case CV_8UC2:
        cv_ptr->encoding = "8UC2";
        break;
      case CV_8UC3:
        cv_ptr->encoding = "8UC3";
        break;
      case CV_8UC4:
        cv_ptr->encoding = "8UC4";
        break;
      case CV_8SC1:
        cv_ptr->encoding = "8SC1";
        break;
      case CV_8SC2:
        cv_ptr->encoding = "8SC2";
        break;
      case CV_8SC3:
        cv_ptr->encoding = "8SC3";
        break;
      case CV_8SC4:
        cv_ptr->encoding = "8SC4";
        break;
      case CV_16UC1:
        cv_ptr->encoding = "16UC1";
        break;
      case CV_16UC2:
        cv_ptr->encoding = "16UC2";
        break;
      case CV_16UC3:
        cv_ptr->encoding = "16UC3";
        break;
      case CV_16UC4:
        cv_ptr->encoding = "16UC4";
        break;
      case CV_16SC1:
        cv_ptr->encoding = "16SC1";
        break;
      case CV_16SC2:
        cv_ptr->encoding = "16SC2";
        break;
      case CV_16SC3:
        cv_ptr->encoding = "16SC3";
        break;
      case CV_16SC4:
        cv_ptr->encoding = "16SC4";
        break;
      case CV_32SC1:
        cv_ptr->encoding = "32SC1";
        break;
      case CV_32SC2:
        cv_ptr->encoding = "32SC2";
        break;
      case CV_32SC3:
        cv_ptr->encoding = "32SC3";
        break;
      case CV_32SC4:
        cv_ptr->encoding = "32SC4";
        break;
      case CV_32FC1:
        cv_ptr->encoding = "32FC1";
        break;
      case CV_32FC2:
        cv_ptr->encoding = "32FC2";
        break;
      case CV_32FC3:
        cv_ptr->encoding = "32FC3";
        break;
      case CV_32FC4:
        cv_ptr->encoding = "32FC4";
        break;
      case CV_64FC1:
        cv_ptr->encoding = "64FC1";
        break;
      case CV_64FC2:
        cv_ptr->encoding = "64FC2";
        break;
      case CV_64FC3:
        cv_ptr->encoding = "64FC3";
        break;
      case CV_64FC4:
        cv_ptr->encoding = "64FC4";
        break;
      default:
        assert(0);
    }

  }

  // Publish message to user callback
  user_cb(cv_ptr->toImageMsg());
}

} //namespace compressed_image_transport
