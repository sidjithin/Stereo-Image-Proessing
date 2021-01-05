#include "stereo_image_proc/point_cloud2.h"

namespace stereo_image_proc {

void PointCloud2Nodelet::onInit()
{

  // Ros nodehandle variable for  PointCloud2Nodelet class 
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection. Optionally do approximate synchronization. 
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    // Approximate time based synching to imageCb callback functions 
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_info_, sub_disparity_) );
    approximate_sync_->registerCallback(boost::bind(&PointCloud2Nodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    // Exact time based synching to imageCb callback functions 
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_info_, sub_disparity_) );
    exact_sync_->registerCallback(boost::bind(&PointCloud2Nodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Monitor whether anyone is subscribed to the output 
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloud2Nodelet::connectCb, this);
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_points2_  = nh.advertise<PointCloud2>("points2",  1, connect_cb, connect_cb);
}

 // Handles (un)subscribing when clients (un)subscribe 
void PointCloud2Nodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_points2_.getNumSubscribers() == 0)
  {
    sub_l_image_  .unsubscribe();
    sub_l_info_   .unsubscribe();
    sub_r_info_   .unsubscribe();
    sub_disparity_.unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Message buffer queue size being provided as 1 
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_  .subscribe(*it_, "left/image_rect_color", 1, hints);
    sub_l_info_   .subscribe(nh,   "left/camera_info", 1);
    sub_r_info_   .subscribe(nh,   "right/camera_info", 1);
    sub_disparity_.subscribe(nh,   "disparity", 1);
  }
}

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z) and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void PointCloud2Nodelet::imageCb(const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg)
{
  // Update the camera model 
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Calculate point cloud 
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud2 message (2D image-like layout) 
  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  points_msg->header = disp_msg->header;
  points_msg->height = mat.rows;
  points_msg->width  = mat.cols;
  points_msg->is_bigendian = false;
  points_msg->is_dense = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

  // Iterate through each of the point cloud,check for validity and assign to the cordinate varaiables
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  for (int v = 0; v < mat.rows; ++v)
  {
    for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      if (isValidPoint(mat(v,u)))
      {
        // x,y,z
        *iter_x = mat(v, u)[0];
        *iter_y = mat(v, u)[1];
        *iter_z = mat(v, u)[2];
      }
      else
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
    }
  }

  // Fill in color 
  namespace enc = sensor_msgs::image_encodings;
  const std::string& encoding = l_image_msg->encoding;
  if (encoding == enc::MONO8)
  {
    const cv::Mat_<uint8_t> color(l_image_msg->height, l_image_msg->width,
                                  (uint8_t*)&l_image_msg->data[0],
                                  l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        uint8_t g = color(v,u);
        *iter_r = *iter_g = *iter_b = g;
      }
    }
  }
  // Check for encoding on sensor_msgs/image RGB8
  else if (encoding == enc::RGB8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& rgb = color(v,u);
        *iter_r = rgb[0];
        *iter_g = rgb[1];
        *iter_b = rgb[2];
      }
    }
  }
  // Check for encoding on sensor_msgs/image BGR8
  else if (encoding == enc::BGR8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& bgr = color(v,u);
        *iter_r = bgr[2];
        *iter_g = bgr[1];
        *iter_b = bgr[0];
      }
    }
  }
  else
  {
    NODELET_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                          "unsupported encoding '%s'", encoding.c_str());
  }

  pub_points2_.publish(points_msg);
}

}   // namespace stereo_image_proc 

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::PointCloud2Nodelet,nodelet::Nodelet)
