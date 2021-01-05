#include "stereo_image_proc/disparity.h"

namespace stereo_image_proc {

void DisparityNodelet::onInit()
{

  ros::NodeHandle &nh = getNodeHandle(); // Ros nodehandle variable
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh)); //Image transport variabale based on nodel handle for ros sensor_msgs/Image topic.
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  //Optional approximate synchronization QueueSize for boost binding for approaximate or exact sync.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  
  if (approx) // Approximate time based synching for imageCb with input message filter subscription topics.
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_image_, sub_r_info_) );
    approximate_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    // Exact time based synching for imageCb,imageCb_boundingBox,traingulate_bounding_box with input message filter subscription topics. 

    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_image_, sub_r_info_) );
    exact_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                              this, _1, _2, _3, _4));


    exact_sync_box.reset( new ExactSync_box(ExactPolicy_box(6),
                                            sub_l_image_,sub_l_info_,
                                            sub_r_info_,sub_disparity_box,sub_box) );
    exact_sync_box->registerCallback(boost::bind(&DisparityNodelet::imageCb_boundingBox,this, _1, _2, _3, _4,_5));



    exact_sync_triang.reset( new ExactSync_triang(ExactPolicy_triang(6),
                                       sub_l_image_, sub_l_info_,
                                       sub_r_image_, sub_r_info_,sub_box) );

    exact_sync_triang->registerCallback(boost::bind(&DisparityNodelet::traingulate_bounding_box,
                                                this, _1, _2, _3, _4,_5));


  }

  // Set up dynamic reconfiguration 
  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
                                                  this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output 
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  // Mutex the Disparity Image Topic 
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
}


 // Handles (un)subscribing when clients (un)subscribe 
void DisparityNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
    sub_box.unsubscribe();
    sub_disparity_box.unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_,  "left/image_rect", 1, hints);
    sub_l_info_ .subscribe(nh,    "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
    sub_box.subscribe(nh,          "boundingBoxes_topic", 1);
    sub_disparity_box.subscribe(nh,      "disparity",1);

  }
}

cv::Mat DisparityNodelet::bounding_center(stereo_image_proc::BoundingBox box_){
    // cv::Mat array for storing the bounding center 
    cv::Mat pointsMat1(2, 1, CV_64F);
    pointsMat1.at<double>(0,0) = box_.x + box_.width/2;
    pointsMat1.at<double>(1,0) = box_.y + box_.height/2;
    return pointsMat1;

}

// Update variable fields of reprojection matrix from Stereo Processing Node
/*
  From Springer Handbook of Robotics, p. 524:

       [ Fx    0  Cx   0   ]
  P  = [ 0     Fy Cy   0   ]
       [ 0     0  1    0   ]

       [ Fx    0  Cx' FxTx ]
  P' = [ 0     Fy Cy   0    ]
       [ 0     0  1    0    ]
  where primed parameters are from the left projection matrix, unprimed from the right.

  [u   v 1]^T = P  * [x y z 1]^T
  [u-d v 1]^T = P' * [x y z 1]^T

  Combining the two equations above results in the following equation

  [u v u-d 1]^T = [ Fx   0    Cx   0    ] * [ x y z 1]^T
                  [ 0    Fy   Cy   0    ]
                  [ Fx   0    Cx'  FxTx ]
                  [ 0    0    1    0    ]

  Subtracting the 3rd from from the first and inverting the expression
  results in the following equation.

  [x y z 1]^T = Q * [u v d 1]^T

  Where Q is defined as

  Q = [ FyTx  0     0   -FyCxTx     ]
      [ 0     FxTx  0   -FxCyTx     ]
      [ 0     0     0    FxFyTx     ]
      [ 0     0     -Fy  Fy(Cx-Cx') ]

 Using the assumption Fx = Fy Q can be simplified to the following. But for
 compatibility with stereo cameras with different focal lengths we will use
 the full Q matrix.

      [ 1 0   0      -Cx      ]
  Q = [ 0 1   0      -Cy      ]
      [ 0 0   0       Fx      ]
      [ 0 0 -1/Tx (Cx-Cx')/Tx ]

  Disparity = x_left - x_right

  For compatibility with stereo cameras with different focal lengths we will use
  the full Q matrix.

 */


void DisparityNodelet::imageCb_boundingBox(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                                           const CameraInfoConstPtr& r_info_msg,const DisparityImageConstPtr& disp_msg,
                                           const stereo_image_proc::BoundingBoxesConstPtr& boxes)
{
    // Variable Bounding box items for left and right cones 
    stereo_image_proc::BoundingBox boundingBox_left;
    stereo_image_proc::BoundingBox boundingBox_right;

    // Looping throught the bounding box items 
    for(auto bounding_box: boxes->bounding_boxes){
         // Based on the image_type within the message structure identify the bounding box from left or right image.
         switch(bounding_box.image_type) {
                       case 0:{ boundingBox_left=bounding_box;}
                         break;
                       case 1:{ boundingBox_right=bounding_box;}
                         break;
                     }
       }

     cv::Mat pointsMat_left(2, 1, CV_64F);
     pointsMat_left = bounding_center(boundingBox_left);
     // Store the disparity image to sensor_msgs/Image Message 
     const Image& dimage = disp_msg->image;
     const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
     //Obtaining the disparity value for the bounding box left point and storing to a float variable. 
     float disparity_cone = dmat.at<float>(pointsMat_left.at<double>(0,0),pointsMat_left.at<double>(1,0));
     //Populating the STEREO Camera model from the camera info message which would eventualy initilaize the Q Matrix required for Reprojection.
     model_.fromCameraInfo(l_info_msg, r_info_msg);
     // Generate the cv::Point2d for providing input for trinagulation 
     cv::Point2d left_uv_rect;
     left_uv_rect.x=pointsMat_left.at<double>(0,0); left_uv_rect.y=pointsMat_left.at<double>(1,0);

     cv::Point3d xyz;
     // Stereo model based reprojection based on q matrix to generate 3d point
     model_.projectDisparityTo3d(left_uv_rect,disparity_cone,xyz);

     std::cout<<" the 3d point is :  "<< xyz.x<<"  "<<xyz.y<<"  "<<xyz.z<<std::endl;


}


void DisparityNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
      // Update the camera model 
      model_.fromCameraInfo(l_info_msg, r_info_msg);

      //Allocate new disparity image message 
      DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
      disp_msg->header         = l_info_msg->header;
      disp_msg->image.header   = l_info_msg->header;

      //Compute window of (potentially) valid disparities 
      int border   = block_matcher_.getCorrelationWindowSize() / 2;
      int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
      int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
      int right  = disp_msg->image.width - 1 - wtf;
      int top    = border;
      int bottom = disp_msg->image.height - 1 - border;
      disp_msg->valid_window.x_offset = left;
      disp_msg->valid_window.y_offset = top;
      disp_msg->valid_window.width    = right - left;
      disp_msg->valid_window.height   = bottom - top;

      // Create cv::Mat views onto all buffers 
      const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
      const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

      // Perform block matching to find the disparities 
      block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

      // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
      double cx_l = model_.left().cx();
      double cx_r = model_.right().cx();
      if (cx_l != cx_r) {
        cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                                  reinterpret_cast<float*>(&disp_msg->image.data[0]),
                                  disp_msg->image.step);
        cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
      }
      // Disparity publishing to ros topic 
      pub_disparity_.publish(disp_msg);
}


void DisparityNodelet::traingulate_bounding_box(const ImageConstPtr& l_image_msg,const CameraInfoConstPtr& l_info_msg,
                                                const ImageConstPtr& r_image_msg,const CameraInfoConstPtr& r_info_msg,
                                                const stereo_image_proc::BoundingBoxesConstPtr& boxes){

    // Converting the ROS Sensor_msgs/Image images to OpenCV Mat format.
    cv::Mat leftImage = cv_bridge::toCvShare(l_image_msg, "mono8")->image;
    cv::Mat rightImage = cv_bridge::toCvShare(r_image_msg, "mono8")->image;

    // Check for the Mat image content.
    if (leftImage.empty() || rightImage.empty()){
    std::cout<<"error cv_bridge"<<std::endl;
    return;
    }
    // Variable for storing the bounding box from left and right outputs.
    stereo_image_proc::BoundingBox boundingBox_left;
    stereo_image_proc::BoundingBox boundingBox_right;

    // Looping throught the bounding box items 
    for(auto bounding_box: boxes->bounding_boxes){
        // Based on the image_type within the message structure identify the bounding box from left or right image.
        switch(bounding_box.image_type) {
          case 0:{ boundingBox_left=bounding_box;}
            break;
          case 1:{ boundingBox_right=bounding_box;}
            break;
        }

    cv::Mat pointsMat_left(2, 1, CV_64F);
    cv::Mat pointsMat_right(2, 1, CV_64F);

    // Calculate the bounding box center pivotal point for both bounding boxes and store in Mat array. 
    pointsMat_left =bounding_center(boundingBox_left);
    pointsMat_right =bounding_center(boundingBox_right);

    // Declare the 3D Homogeneous for storing the output 
    cv::Mat pnts3D(4, 1, CV_32F);

    // Declare the Projection Matrix array for storing the camera info_left->P Matrix 
    float P1_array[12];

    cv::Mat P1;
    for (int i = 0; i < 12; i++) {
    P1_array[i] = l_info_msg->P[i];
    }
    // Converting the Float array to cv::Mat array 
    P1 = cv::Mat(3, 4, CV_32F, &P1_array[0]);

    // Declare the Projection Matrix array for storing the camera info right->P Matrix 
    float P2_array[12];

    cv::Mat P2;
    for (int i = 0; i < 12; i++) {
    P2_array[i] = r_info_msg->P[i];
    }
    // Converting the Float array to cv::Mat array 
    P2 = cv::Mat(3, 4, CV_32F, &P2_array[0]);

    // Triangulating the corresponding points from left and right camera using their projection matrix. 
    cv::triangulatePoints(P1, P2, pointsMat_left, pointsMat_right, pnts3D);

    // Obtaining the 3d cartesian cordinate from the 3D HOmogeneous cordinate. 
    cv::Point3d point3D_cone;
    point3D_cone.x = pnts3D.at<double>(0, 0);
    point3D_cone.y = pnts3D.at<double>(1, 0);
    point3D_cone.z = pnts3D.at<double>(2, 0);

    // Final 3D cartesian cordinate for the cone position. 
    point3D_cone.x = point3D_cone.x/pnts3D.at<double>(3, 0);
    point3D_cone.y = point3D_cone.y/pnts3D.at<double>(3, 0);
    point3D_cone.z = point3D_cone.z/pnts3D.at<double>(3, 0);


}

}


/*!
 * \brief projectDisparityTo3d : Function that reprojects the 2d image_rect to 3d point.
 * \param left_uv_rect : Rectified image cordinates for which 3d reprojected values to be determined.
 * \param disparity    : Disparity value for the corresponding point.
 * \param xyz          : Output xyz point in cartesian cordinate.
 */
void projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity,
                                             cv::Point3d& xyz)
{
    cv::Matx44d Q_;

  // Do the math inline:
  // [X Y Z W]^T = Q * [u v d 1]^T
  // Point = (X/W, Y/W, Z/W)
  // cv::perspectiveTransform could be used but with more overhead.
  double u = left_uv_rect.x, v = left_uv_rect.y;
  cv::Point3d XYZ( (Q_(0,0) * u) + Q_(0,3), (Q_(1,1) * v) + Q_(1,3), Q_(2,3));
  // Reprojection Matrix Q. 
  double W = Q_(3,2)*disparity + Q_(3,3);
  xyz = XYZ * (1.0/W);

}



void DisparityNodelet::configCb(Config &config, uint32_t level)
{
  // Tweak all settings to be valid 
  config.prefilter_size |= 0x1;
  config.correlation_window_size |= 0x1;
  // must be multiple of 16 
  config.disparity_range = (config.disparity_range / 16) * 16;
  
  // Block Matching dynamic variable settings 
  block_matcher_.setPreFilterCap(config.prefilter_cap);
  block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
  block_matcher_.setMinDisparity(config.min_disparity);
  block_matcher_.setDisparityRange(config.disparity_range);
  block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
  block_matcher_.setSpeckleSize(config.speckle_size);
  block_matcher_.setSpeckleRange(config.speckle_range);
  if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
    block_matcher_.setStereoType(StereoProcessor::BM);
    block_matcher_.setPreFilterSize(config.prefilter_size);
    block_matcher_.setTextureThreshold(config.texture_threshold);
  }
  else if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
    block_matcher_.setStereoType(StereoProcessor::SGBM);
    block_matcher_.setSgbmMode(config.fullDP);
    block_matcher_.setP1(config.P1);
    block_matcher_.setP2(config.P2);
    block_matcher_.setDisp12MaxDiff(config.disp12MaxDiff);
  }
}

}
 // end of stereo_image_proc namespace 

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::DisparityNodelet,nodelet::Nodelet)
