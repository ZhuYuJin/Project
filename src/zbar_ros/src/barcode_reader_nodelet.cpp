/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "zbar_ros/barcode_reader_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

namespace zbar_ros
{

  BarcodeReaderNodelet::BarcodeReaderNodelet()
  {
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  void BarcodeReaderNodelet::onInit()
  {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    barcode_pub_ = nh_.advertise<std_msgs::String>("barcode", 10,
        boost::bind(&BarcodeReaderNodelet::connectCb, this),
        boost::bind(&BarcodeReaderNodelet::disconnectCb, this));
    
    private_nh_.param<double>("throttle_repeated_barcodes", throttle_, 0.0);
    if (throttle_ > 0.0){
      clean_timer_ = nh_.createTimer(ros::Duration(10.0), boost::bind(&BarcodeReaderNodelet::cleanCb, this));
    }
  };

  void BarcodeReaderNodelet::connectCb()
  {
    if (!camera_sub_ && barcode_pub_.getNumSubscribers() > 0)
    {
      NODELET_INFO("Connecting to camera topic.");
      camera_sub_ = nh_.subscribe("/usb_cam/image_raw", 10, &BarcodeReaderNodelet::imageCb, this);
    }
  }

  void BarcodeReaderNodelet::disconnectCb()
  {
    if (barcode_pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("Unsubscribing from camera topic.");
      camera_sub_.shutdown();
    }
  }

  void BarcodeReaderNodelet::imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat img;
    cvtColor(cv_ptr->image, img, CV_BGR2GRAY);  

// cv::Mat img = cv::imread("/home/zhuyujin/Downloads/test3.jpg", CV_LOAD_IMAGE_GRAYSCALE);

// zbar::Image zbar_image(img.cols, img.rows, "Y800", img.data,
       // img.cols * img.rows);
// scanner_.scan(zbar_image);

    zbar::Image zbar_image(img.cols, img.rows, "Y800", img.data,
        img.cols * img.rows);
    scanner_.scan(zbar_image);

    // iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol)
    {
      std::string barcode = symbol->get_data();

/* 
 *    location of barcode
 */
      int x = 0;
      int y = 0;
      int x_min = 999999;
      int x_max = 0;
      int y_min = 999999;
      int y_max = 0;
      int size = symbol->get_location_size();
      int x_loc, y_loc;
      int x1, y1, x2, y2, x3, y3, x4, y4;
      for(int i = 0; i < size; i++){
        x_loc = symbol->get_location_x(i);
        y_loc = symbol->get_location_y(i);
        if(i == 0){ x1 = x_loc; y1 = y_loc; }
        if(i == 1){ x2 = x_loc; y2 = y_loc; }
        if(i == 2){ x3 = x_loc; y3 = y_loc; }
        if(i == 3){ x4 = x_loc; y4 = y_loc; }
        x += x_loc;
        y += y_loc;
        if(x_loc < x_min) x_min = x_loc;
        if(x_loc > x_max) x_max = x_loc;
        if(y_loc < y_min) y_min = y_loc;
        if(y_loc > y_max) y_max = y_loc;
      }
  // int width = zbar_image.get_width();
  // int height = zbar_image.get_height();

      ROS_INFO("[mid_x:%d mid_y:%d x_min:%d x_max:%d y_min:%d y_max:%d x_1:%d y_1:%d x_2:%d y_2:%d x_3:%d y_3:%d x_4:%d y_4:%d]", x/size, y/size, x_min, x_max, y_min, y_max, x1, y1, x2, y2, x3, y3, x4, y4);
/* 
 *    location of barcode
 */

      // verify if repeated barcode throttling is enabled
      if (throttle_ > 0.0)
      {
        // check if barcode has been recorded as seen, and skip detection
        if (barcode_memory_.count(barcode) > 0)
        {
          // check if time reached to forget barcode
          if (ros::Time::now() > barcode_memory_.at(barcode))
          {
            NODELET_DEBUG("Memory timed out for barcode, publishing");
            barcode_memory_.erase(barcode);
          }
          else
          {
            // if timeout not reached, skip this reading
            continue;
          }
        }
        // record barcode as seen, with a timeout to 'forget'
        barcode_memory_.insert(std::make_pair(barcode, ros::Time::now() + ros::Duration(throttle_)));
      }

      // publish barcode
      stringstream ss;
      ss << x/size << " ";
      ss << y/size << " ";
      ss << x_min << " ";
      ss << x_max << " ";
      ss << y_min << " ";
      ss << y_max << " ";
      ss << x1 << " ";
      ss << y1 << " ";
      ss << x2 << " ";
      ss << y2 << " ";
      ss << x3 << " ";
      ss << y3 << " ";
      ss << x4 << " ";
      ss << y4 << " ";
      ss << barcode;

      std_msgs::String barcode_string;
      barcode_string.data = ss.str();
      barcode_pub_.publish(barcode_string);
    }
  }

  void BarcodeReaderNodelet::cleanCb()
  {
    for (boost::unordered_map<std::string, ros::Time>::iterator it = barcode_memory_.begin();
         it != barcode_memory_.end(); ++it)
    {
      if (ros::Time::now() > it->second)
      {
        NODELET_DEBUG_STREAM("Cleaned " << it->first << " from memory");
        barcode_memory_.erase(it);
      }
    }

  }
}  // namespace zbar_ros

PLUGINLIB_DECLARE_CLASS(zbar_ros, BarcodeReaderNodelet, zbar_ros::BarcodeReaderNodelet, nodelet::Nodelet);
