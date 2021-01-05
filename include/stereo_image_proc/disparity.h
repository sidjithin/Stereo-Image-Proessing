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
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
#include <dynamic_reconfigure/server.h>

#include <stereo_image_proc/processor.h>
#include <stereo_image_proc/BoundingBoxes.h>
#include <stereo_image_proc/BoundingBox.h>

/*!
 * Namespace stereo_image_proc for computing disparity and 3d pointcloud from stereo images.
 */

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

/*!
 * \brief The DisparityNodelet class :Class that perform the disparity calculation and perform the bounding box based calculations.
 *        Message filters being used to synchronize the ros data topics.Exact syncronization or Approximate synchrnoization used.
 */
class DisparityNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  

  /*!
   * \brief Subscriptions message filter topics for left image,right image,left camera info,right camera info ,bouding box and disparity.
   */
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<stereo_image_proc::BoundingBoxes> sub_box;
  message_filters::Subscriber<DisparityImage> sub_disparity_box;


  /*!
   * \brief Synchronization policy for callback functions
   */
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

  typedef ExactTime<Image, CameraInfo,CameraInfo,DisparityImage,stereo_image_proc::BoundingBoxes> ExactPolicy_box;
  typedef message_filters::Synchronizer<ExactPolicy_box> ExactSync_box;

  typedef ExactTime<Image, CameraInfo, Image, CameraInfo,stereo_image_proc::BoundingBoxes> ExactPolicy_triang;
  typedef message_filters::Synchronizer<ExactPolicy_triang> ExactSync_triang;


  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;


  /*!
   * \brief Boost shared pointer for callback parameter Boost binding.
   */
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ExactSync_box> exact_sync_box;
  boost::shared_ptr<ExactSync_triang> exact_sync_triang;

  boost::shared_ptr<ApproximateSync> approximate_sync_;
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_;

  /*!
   * \brief Dynamic reconfigurable parameters for stereo image processing for Block Matching or SG Block matching.
   * Prameters for prefilter_size,prefilter_cap,correlation_window_size,min_disparity,disparity_range,uniqueness_ratio,texture_threshold,speckle_size,speckle_range.
   */
  boost::recursive_mutex config_mutex_;
  typedef stereo_image_proc::DisparityConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  /*!
   * \brief model_:Stereo model variable that gets filled with the camera information from left and right camera.
   */
  image_geometry::StereoCameraModel model_;

  /*!
   * \brief block_matcher_  variable that contains scratch buffers for block matching.
   */
  stereo_image_proc::StereoProcessor block_matcher_;


  /*!
   * \brief onInit :Function that initialises the ros::NodeHandle ,subscription topics with their relevant callback functions.
   */
  virtual void onInit();
  /*!
   * \brief connectCb:Funtion that connect the message filter subscription variable to their input ROS topics.
   */
  void connectCb();


  /*!
   * \brief imageCb: Function that performs the stereo processing to generate the disparity image from stereo pairs and publishes.
   * \param l_image_msg : Left camera image
   * \param l_info_msg  : left camera info.
   * \param r_image_msg : Right camera image.
   * \param r_info_msg  : Right camera info.
   */
  void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
               const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

  /*!
   * \brief imageCb_boundingBox :Function that calculates the 3D points for the bounding box,the computed bounding box are generated with respect to the left camera cordinate frame.
   * \param l_image_msg:Left camera image
   * \param l_info_msg :left camera info.
   * \param r_info_msg :Right camera info.
   * \param disp_msg   :Disparity image generated from imageCb callback method.
   * \param boxes      :Bounding boxes topic following stereo_image_proc/BoundingBoxes message structure.
   */
  void imageCb_boundingBox(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                      const CameraInfoConstPtr& r_info_msg,const DisparityImageConstPtr& disp_msg,const stereo_image_proc::BoundingBoxesConstPtr& boxes);

  /*!
   * \brief configCb : Function that initilaises the block matching parameters.
   * \param config   : The config Parameter from dynamic reconfigure.
   * \param level    : Parameter level.
   */
  void configCb(Config &config, uint32_t level);
  /*!
   * \brief traingulate_bounding_box :Fucntion that perform OpenCV traingulation method from two corresponding points from the stereo_image_proc/BoundingBoxes topic.
   * \param l_image_msg :Left camera image
   * \param l_info_msg  :left camera info.
   * \param r_image_msg :Right camera image.
   * \param r_info_msg  :Right camera info.
   * \param boxes       :Bounding boxes topic following stereo_image_proc/BoundingBoxes message structure.
   */
  void traingulate_bounding_box(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                                const ImageConstPtr& r_image_msg,const CameraInfoConstPtr& r_info_msg,
                                const stereo_image_proc::BoundingBoxesConstPtr& boxes);
  /*!
   * \brief bounding_center :Function that calulates the center point of the bounding box.
   * \param box_ :The paramaters box message.
   * \return     :cv::Mat array of 1*2 array.
   */
  cv::Mat bounding_center(stereo_image_proc::BoundingBox box_);
};
}
