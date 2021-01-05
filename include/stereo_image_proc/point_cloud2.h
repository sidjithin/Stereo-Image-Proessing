/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
/*!
 * \brief The PointCloud2Nodelet class: Point cloud based processing.
 */
class PointCloud2Nodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;



  /*!
   * \brief Subscriptions for subscribing left image and camera info topics for left and right topics.
   */
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;

  /*!
   * \brief ExactPolicy :Synchronozing policy for Camera Image,Camera info, disparity topics.
   */
  typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  /*!
   * \brief exact_sync_ :Boost shared pointer for binding the input subscribed topics.
   */
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_points2_;

  /*!
   * \brief model_:Stereo model variable that gets filled with the camera information from left and right camera.
   */
  image_geometry::StereoCameraModel model_;
  /*!
   * \brief points_mat_ :scratch buffer for storing Point Cloud
   */
  cv::Mat_<cv::Vec3f> points_mat_;
  
  /*!
   * \brief onInit :Initialize the susbcription topics with corresponding callback functions.
   */
  virtual void onInit();

  /*!
   * \brief connectCb :Funtion that connect the message filter subscription variable to their input ROS topics.
   */
  void connectCb();


  /*!
   * \brief imageCb    :Function that reprojects the entire disparity image to pointcloud data.
   * \param l_image_msg:Left camera info message.
   * \param l_info_msg :Right camera info message.
   * \param r_info_msg :Right camera image
   * \param disp_msg   :Disparity image from DisparityNodelet class.
   */
  void imageCb(const ImageConstPtr& l_image_msg,
               const CameraInfoConstPtr& l_info_msg,
               const CameraInfoConstPtr& r_info_msg,
               const DisparityImageConstPtr& disp_msg);
};
}
