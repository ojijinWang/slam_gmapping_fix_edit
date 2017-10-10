/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"
#include "map_loader.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping_fix/sensor/sensor_range/rangesensor.h"
#include "gmapping_fix/sensor/sensor_odometry/odometrysensor.h"

#include <gmapping_fix/utils/macro_params.h>
#include <gmapping_fix/utils/stat.h>
#include <gmapping_fix/utils/gvalues.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
double d_Rand();
void pose2tansform(const geometry_msgs::Pose& pose_in, tf::Transform& transform_out);
void tansform2pose(const tf::Transform& transform_in, geometry_msgs::Pose& pose_out);

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}


void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping_Fix::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;
  got_init_map_ = false;
  got_init_pose_ = false;
  is_init_pose_param_ = false;
  is_the_firstSacn_after_init_map = false;
  is_the_secondScan_after_init_map = false;
  
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
  
  //get init pose form params
  is_init_pose_param_ = true;
  if(!private_nh_.getParam("initial_pose_x", initial_pose_x_))
  { is_init_pose_param_ = false; }
  if(!private_nh_.getParam("initial_pose_y", initial_pose_y_))
  { is_init_pose_param_ = false; }
  if(!private_nh_.getParam("initial_pose_a", initial_pose_a_))
  { is_init_pose_param_ = false; }
  if(!private_nh_.getParam("initial_cov_xx", initial_cov_xx_))
  { is_init_pose_param_ = false; }
  if(!private_nh_.getParam("initial_cov_yy", initial_cov_yy_))
  { is_init_pose_param_ = false; }
  if(!private_nh_.getParam("initial_cov_aa", initial_cov_aa_))
  { is_init_pose_param_ = false; }
  if(!is_init_pose_param_){
	initial_pose_x_= 0;
	initial_pose_y_= 0;
	initial_pose_a_= 0;
	initial_cov_xx_= 0.09;
	initial_cov_yy_= 0.09;
	initial_cov_aa_= 0.04;
  }
  else { printf("start SLAM with init position\n"); }

  //get init map form params
  bool is_init_map_params = true;
  if(!private_nh_.getParam("map_yaml_file_name", yaml_file_name_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("map_pgm_file_name", map_file_name_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("image", image_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("resolution", resolution_)) 
  { is_init_map_params = false; }
  if(!private_nh_.getParam("origin", origin_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("negate", negate_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("occupied_thresh", occupied_thresh_))
  { is_init_map_params = false; }
  if(!private_nh_.getParam("free_thresh", free_thresh_))
  { is_init_map_params = false; }

  //we should not use the map if the position is not initialized
  if(is_init_map_params)
  { printf("start SLAM with init map\n"); }

  // start initialized ability
  if(is_init_map_params)
  {
	read_map_formFile(yaml_file_name_, init_map_param_, init_mapMetaData_param_);
	initMap(init_map_param_);
  }
  if(is_init_pose_param_)
  {
	geometry_msgs::Pose pose_now;
	pose_now.position.x = initial_pose_x_;
	pose_now.position.y = initial_pose_y_;
	pose_now.position.z = 0.0;
	tf::Quaternion q;
	q.setRPY(0, 0, initial_pose_a_);
	geometry_msgs::Quaternion q_msg;
	tf::quaternionTFToMsg(q, q_msg);
	pose_now.orientation = q_msg;
	initPose(pose_now);
	is_the_firstSacn_after_init_map = true;

  }

  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  paticleSample_pub = node_.advertise<geometry_msgs::PoseArray>("particles_poses", 1, true);
  slam_condition_pub = node_.advertise<std_msgs::Int32>("gammping_condition", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 1);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 1);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  /******************My changed part start**********************/
  // add a sub to read map data
  chatter_mapInitCallback_sub = node_.subscribe("gmapping_init_map", 1, &SlamGMapping::mapInitCallback, this);
  chatter_poseInitCallback_sub = node_.subscribe("gmapping_init_pose", 1, &SlamGMapping::poseInitCallback, this);
  chatter_mapPoseInitCallback_sub = node_.subscribe("gmapping_init_mapPose", 1, &SlamGMapping::mapPoseInitCallback, this);
  chatter_poseCovInitCallback_sub = node_.subscribe("gmapping_init_poseCov", 1, &SlamGMapping::poseCovInitCallback, this);
  chatter_poseCovEKFCallback_sub = node_.subscribe("gmapping_ekf_poseCov", 1, &SlamGMapping::poseCovEKFCallback, this);
  /******************My changed part end************************/
  printf("gmapping program start operation!\n");

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}

void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  int count = 0;
  while(ros::ok())
  {
    publishTransform();
    if( count > 20 )	//publish particle poses PoseArray
    { count = 0; publishParticleSample(); }
    count ++;
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete smap_init_;
  delete smap_init_occupied_;
  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool SlamGMapping::getOdomPose(GMapping_Fix::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping_Fix::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  //publish the initial condition; 1
  std_msgs::Int32 condition;
  condition.data = CONDITION_INITICAL;
  slam_condition_pub.publish(condition);

  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    //tf_.transformPose(base_frame_, ident, laser_pose);
    tf_.transformPose(base_frame_, ros::Time(0), ident, ident.frame_id_, laser_pose);
    tf_.lookupTransform(base_frame_, laser_frame_, ros::Time(0), transform_base2laser);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping_Fix::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;
  if(!private_nh_.getParam("minRange", minRange_))
    minRange_ = 0.01;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping_Fix::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping_Fix::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping_Fix::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping_Fix::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping_Fix::OrientedPoint(0.0, 0.0, 0.0);
  }
  if(is_init_pose_param_)
  {
	is_init_pose_param_ = false;

	//get laser pose form base_link pose;
	//get transform
	tf::Transform transform_map2base;
	transform_map2base.setOrigin(tf::Vector3(initial_pose_x_, initial_pose_y_, 0.0));
	tf::Quaternion q_base;
	q_base.setRPY(0.0, 0.0, initial_pose_a_);
	transform_map2base.setRotation(q_base);
	//get pose form transform
	geometry_msgs::Pose pose_init_laser;
	tf::Transform transform_map2laser = transform_map2base * transform_base2laser;
	tansform2pose(transform_map2laser, pose_init_laser);
	//change pose to x,y,z,roll.pitch.yaw
	geometry_msgs::Point position 		= pose_init_laser.position;
	geometry_msgs::Quaternion orientation 	= pose_init_laser.orientation;
	double x, y, z;
	double roll, pitch, yaw;
	x = position.x;
	y = position.y;
	z = position.z;
	tf::Quaternion q_laser(orientation.x, orientation.y, orientation.z, orientation.w);
	tf::Matrix3x3(q_laser).getRPY(roll, pitch, yaw);
	//update initial position x,y,yaw
	initial_pose_x_ = x;
	initial_pose_y_ = y;
	initial_pose_a_ = yaw;

	initialPose.x = initial_pose_x_;
	initialPose.y = initial_pose_y_;
	initialPose.theta = initial_pose_a_;
  }
  if(got_init_pose_)
  { initialPose = pose_init_; }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  if(got_init_pose_)
  { gsp_->setMotionModelParameters(srr_*5, srt_*20, str_*20, stt_*5); }
  else
  { gsp_->setMotionModelParameters(srr_, srt_, str_, stt_); }
  if(got_init_pose_)
  { gsp_->setUpdateDistances(0.0, 0.0, resampleThreshold_); }
  else
  { gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_); }
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping_Fix::sampleGaussian(1,seed_);

  if(got_init_pose_ && !got_init_map_)
  {
	double error_x = sqrt(initial_cov_xx_);
	double error_y = sqrt(initial_cov_yy_);
	double error_theta = sqrt(initial_cov_aa_);
	gsp_->m_particles[0].pose.x = pose_init_.x;	//take first partile as the best particle: weight = max
	gsp_->m_particles[0].pose.y = pose_init_.y;
	gsp_->m_particles[0].pose.theta = pose_init_.theta;
	gsp_->m_particles[0].weight = 1;
	for(int i=1; i< gsp_->m_particles.size(); i++)
	{
		gsp_->m_particles[i].pose.x = pose_init_.x + error_x * d_Rand();
		gsp_->m_particles[i].pose.y = pose_init_.y + error_y * d_Rand();
		gsp_->m_particles[i].pose.theta = pose_init_.theta  + error_theta * d_Rand();
		gsp_->m_particles[i].weight = 0.1;
	}
  }

  ROS_INFO("Initialization complete");

  return true;
}

bool SlamGMapping::addScan(sensor_msgs::LaserScan& scan, GMapping_Fix::OrientedPoint& gmap_pose) // send scan to make slam
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))
  { return false;}
  
  if(scan.ranges.size() != gsp_laser_beam_count_)
  { return false;}

  //build a fake scan using init_pose_ and init_map_ to init particle when first scan & got_init_map_;
  if(is_the_firstSacn_after_init_map && got_init_map_)
  { 
	sensor_msgs::LaserScan scan_out;
	fakeScanUsingInitMap(scan_out, scan, *smap_init_, pose_init_);
	scan = scan_out;
  }

  // GMapping wants an array of doubles... // ranges_double is the scan will send to peocesser
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping_Fix::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  if(is_the_secondScan_after_init_map)
  { 
	gmap_pose.x += (linearUpdate_ * 2 * d_Rand());
  	gmap_pose.y += (linearUpdate_ * 2 * d_Rand());
  	gmap_pose.theta += (angularUpdate_ * 0 * d_Rand());
  }
  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  bool processScan_result = gsp_->processScan(reading);		//make slam

  /*if(processScan_result)
  { printf("processing scan OK\n"); }
  else
  { printf("processing scan igonre\n"); }*/

  if(is_the_secondScan_after_init_map && processScan_result) //reset the particle file para after init
  { 
	gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
	gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
	is_the_secondScan_after_init_map = false;      
  }
  //update map or postion if the is_the_firstSacn_after_init_map set
  if(is_the_firstSacn_after_init_map)
  {
	is_the_firstSacn_after_init_map = false;
	is_the_secondScan_after_init_map = true;
	if(got_init_map_ && got_init_pose_)
  	{
		for(int i=0; i< gsp_->m_particles.size(); i++)
  		{
			GMapping_Fix::GridSlamProcessor::Particle particle_temp = gsp_->m_particles[i];
			gsp_->m_particles[i] = GMapping_Fix::GridSlamProcessor::Particle(*smap_init_occupied_);
			gsp_->m_particles[i].pose 		= particle_temp.pose;
			gsp_->m_particles[i].previousPose 	= particle_temp.previousPose;
			gsp_->m_particles[i].weight 		= particle_temp.weight;
			gsp_->m_particles[i].weightSum 		= particle_temp.weightSum;
			gsp_->m_particles[i].gweight 		= particle_temp.gweight;
			gsp_->m_particles[i].previousIndex 	= particle_temp.previousIndex;
			gsp_->m_particles[i].node 		= particle_temp.node;
		}
	}
	if(got_init_pose_)
	{
		double error_x = sqrt(initial_cov_xx_);
		double error_y = sqrt(initial_cov_yy_);
		double error_theta = sqrt(initial_cov_aa_);
		gsp_->m_particles[0].pose.x = pose_init_.x;	//take first partile as the best particle: weight = max
		gsp_->m_particles[0].pose.y = pose_init_.y;
		gsp_->m_particles[0].pose.theta = pose_init_.theta;
		gsp_->m_particles[0].weight = 1;
		for(int i=1; i< gsp_->m_particles.size(); i++)
		{
			gsp_->m_particles[i].pose.x = pose_init_.x + error_x * d_Rand();
			gsp_->m_particles[i].pose.y = pose_init_.y + error_y * d_Rand();
			gsp_->m_particles[i].pose.theta = pose_init_.theta  + error_theta * d_Rand();
			gsp_->m_particles[i].weight = 0.1;
		}
	}
  }
  return processScan_result;
}

void SlamGMapping::laserCallback_s(sensor_msgs::LaserScan& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(scan))
    { return; }
    got_first_scan_ = true;
  }

  GMapping_Fix::OrientedPoint odom_pose;

  if(addScan(scan, odom_pose))
  {
    ROS_DEBUG("scan processed");

    GMapping_Fix::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    if(is_the_secondScan_after_init_map || is_the_firstSacn_after_init_map)
    { mpose = pose_init_; }
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);


    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan.header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(scan);
      last_map_update = scan.header.stamp;
      ROS_DEBUG("Updated the map");
      //publish condition that slam is working on;

      if(!is_the_firstSacn_after_init_map && !is_the_secondScan_after_init_map)
      {
		std_msgs::Int32 condition;
		condition.data = CONDITION_WORKING;
		slam_condition_pub.publish(condition);  
      }
    }
  } else
    ROS_DEBUG("cannot process scan");
}

void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scan_last.header = scan->header;
  scan_last.angle_min = scan->angle_min;
  scan_last.angle_max = scan->angle_max;
  scan_last.angle_increment = scan->angle_increment;
  scan_last.time_increment = scan->time_increment;
  scan_last.scan_time = scan->scan_time;
  scan_last.range_min = scan->range_min;
  scan_last.range_max = scan->range_max;
  scan_last.ranges = scan->ranges;
  scan_last.intensities = scan->intensities;

  laserCallback_s(scan_last);
}

double SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping_Fix::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping_Fix::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping_Fix::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping_Fix::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_ && !got_init_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.width = 0;
    map_.map.info.height = 0;
  }

  GMapping_Fix::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping_Fix::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);
  int center_x_pixel = smap.getMapSizeX() / 2;
  int center_y_pixel = smap.getMapSizeY() / 2;

  //add the icp map(black only map) of the best partile
  /*for(int x_cell=0; x_cell< best.map.getMapSizeX(); x_cell++)	//x_cell : pixels
  {
        for(int y_cell=0; y_cell< best.map.getMapSizeY(); y_cell++)
        {
          GMapping_Fix::IntPoint p(x_cell, y_cell);
          double occ = best.map.cell(p);

	  if(occ < 0)
	  { continue; }
          bool is_occupied = false; //ture = black; false = gray
          if(occ < 0 || occ > 1.00000001)		//unknow is data is out of rang
          { occ = -1; continue; }
          else if( occ > occ_thresh_ )	// occupied if occ is bigger than thre-gate
          { occ = 1; is_occupied = true; }
          else if( occ < 0.2 )		// empty if occ is smaller than thre-gate	//FIXME the low-threhold should be determin perfer!
          { occ = 0; is_occupied = false; } 
          else
          { continue; }
        
          //get the position with meter
          double x_meter = (x_cell - center_x_pixel) * delta_;
          double y_meter = (y_cell - center_y_pixel) * delta_;

          // add data into possility map
          matcher.invalidateActiveArea();
          matcher.computeActivePoint(smap, x_meter, y_meter);
          matcher.registerPoint(smap, x_meter, y_meter, is_occupied);
          // add data into possility map which has occupied information only;
        }
  }*/

  ROS_DEBUG("Trajectory tree:");
  for(GMapping_Fix::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));	//n is the best particle, smap is the map saved in the best particle.
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) 
  {
    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping_Fix::Point wmin = smap.map2world(GMapping_Fix::IntPoint(0, 0));
    GMapping_Fix::Point wmax = smap.map2world(GMapping_Fix::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
    got_map_ = false;

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  /******************My changed part start**********************/
  //clear map if this is the first scan
  if(!got_map_)
  {
	int buffer_size = map_.map.info.width * map_.map.info.width;
	for(int i=0; i < buffer_size; i++)
  	{ map_.map.data[i] = -1; }
  }
  //set global map if this is the first scan & got_init_map_
  if(got_init_map_ && got_init_pose_ && !got_map_)
  {
	double resolution_init = (*smap_init_).getResolution();				//meter
	int center_x_pixel_init = (*smap_init_).getMapSizeX() / 2;					//pixel
	int center_y_pixel_init = (*smap_init_).getMapSizeY() / 2;					//pixel
	for(int x_cell=0; x_cell< (*smap_init_).getMapSizeX(); x_cell++)	//x_cell : pixels
	{
		for(int y_cell=0; y_cell< (*smap_init_).getMapSizeY(); y_cell++)
		{
			GMapping_Fix::IntPoint p(x_cell, y_cell);
			double occ = (*smap_init_).cell(p);

			if(occ < 0)
			{ continue; }
			bool is_occupied = false; //ture = black; false = gray
			if(occ < 0 || occ > 1.00000001)		//unknow is data is out of rang
			{ occ = -1; continue; }
			else if( occ > occ_thresh_ )	// occupied if occ is bigger than thre-gate
			{ occ = 1; is_occupied = true; }
			else if( occ < 0.2 )		// empty if occ is smaller than thre-gate
			{ occ = 0; is_occupied = false; } 
			else
			{ continue; }
        
			//get the position with meter
			double x_meter = (x_cell - center_x_pixel_init) * resolution_init;
			double y_meter = (y_cell - center_y_pixel_init) * resolution_init;

			//get the pixel position on map to publish -> in case the size of map is diff
			int x_pixel = x_meter / delta_ + center_x_pixel;
			int y_pixel = y_meter / delta_ + center_x_pixel;
			if(occ > occ_thresh_)
      			{ map_.map.data[MAP_IDX(map_.map.info.width, x_pixel, y_pixel)] = 100; }
      			else
      			{ map_.map.data[MAP_IDX(map_.map.info.width, x_pixel, y_pixel)] = 0; }
		}
	}
  }

  //the laser position is the original position of the slam/robot in gmapping
  GMapping_Fix::OrientedPoint laser_pose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
  double laser_pose_x_meter = laser_pose.x;
  double laser_pose_y_meter = laser_pose.y;

  double range_max_pixel   = maxRange_ / delta_;
  double range_min_pixel   = minRange_ / delta_;
  double range_max_pixel_2 = range_max_pixel * range_max_pixel;
  double range_min_pixel_2 = range_min_pixel * range_min_pixel;
  double laser_pose_x_pixel   = laser_pose_x_meter / delta_ + center_x_pixel;
  double laser_pose_y_pixel   = laser_pose_y_meter / delta_ + center_y_pixel;
  int active_area_x_min = laser_pose_x_pixel - range_max_pixel;
  int active_area_x_max = laser_pose_x_pixel + range_max_pixel;
  int active_area_y_min = laser_pose_y_pixel - range_max_pixel;
  int active_area_y_max = laser_pose_y_pixel + range_max_pixel;
  if(active_area_x_min < 0) { active_area_x_min = 0; }
  if(active_area_x_max > smap.getMapSizeX()) { active_area_x_max = smap.getMapSizeX(); }
  if(active_area_y_min < 0) { active_area_y_min = 0; }
  if(active_area_y_max > smap.getMapSizeY()) { active_area_y_max = smap.getMapSizeY(); }
  for(int x=active_area_x_min; x < active_area_x_max; x++)
  {
    for(int y=active_area_y_min; y < active_area_y_max; y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping_Fix::IntPoint p(x, y);
      double occ = smap.cell(p);
      assert(occ <= 1.0);

      double map_x_of_laser = x - laser_pose_x_pixel;
      double map_y_of_laser = x - laser_pose_y_pixel;
      double range_pixel_2 = map_x_of_laser*map_x_of_laser + map_y_of_laser*map_y_of_laser;

      //if(got_map_ && range_pixel_2 > range_max_pixel_2)  //do not update the map which is bigger than laser scan max range -> save time
      //{ continue; }
      //if(got_map_ && range_pixel_2 < range_min_pixel_2 && ( occ < occ_thresh_ && occ > 0 ))  //do not update the map which is smaller than laser scan max range -> laser minRange
      //{ 
	//	if(map_.map.data[MAP_IDX(map_.map.info.width, x, y)] > 0)
	//	{ continue;} 
      //}
      
      if(occ < 0)
      { continue; map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1; }
      else if(occ > occ_thresh_)
      { map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100; }
      else
      { map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0; }
    }
  }

  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );
  //publish occupied map
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

void SlamGMapping::mapInitCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_in)	//initicalize map using income map
{
  //build a new value to copy the initilization map
  nav_msgs::OccupancyGrid map_in_s;
  map_in_s.header = map_in->header;
  map_in_s.info = map_in->info;
  map_in_s.data = map_in->data;
  //init map;
  initMap(map_in_s);
  //FIXME do nothing but only save the map. no init!!! here, should i set it using positions now? what if the area is occupied?
}

void SlamGMapping::poseInitCallback(const geometry_msgs::Pose::ConstPtr& pose_in)
{
  std_msgs::Int32 condition;
  condition.data = CONDITION_INITICAL;
  slam_condition_pub.publish(condition);

  //build a new value to copy the initilization positon
  geometry_msgs::Pose pose_in_s;
  pose_in_s.position = pose_in->position;
  pose_in_s.orientation = pose_in->orientation;
  //get laser pose form base_link pose
  tf::Transform transform_map2base;
  pose2tansform(pose_in_s, transform_map2base);
  tf::Transform transform_map2laser = transform_map2base * transform_base2laser;
  tansform2pose(transform_map2laser, pose_in_s);
  //init pose;
  initPose(pose_in_s);

  //refresh all the setting by using funtion laserCallback_s(Class& scan);
  //FIXME the old map in each particle will be deleted after refreshing. that is not what i want!!!
  if(got_first_scan_){
        is_the_firstSacn_after_init_map = true;
  	got_first_scan_ = false;
	got_map_ = false;
	laserCallback_s(scan_last);
  }
}

void SlamGMapping::mapPoseInitCallback(const gmapping_fix::gmapping_initMapPoseMsg::ConstPtr& map_pose_in)	//initicalize map using income map & income position
{
  //in this funtion:
  //1. save pose_init_
  //2. save smap_init_
  //3. set the flages
  //4. clear the flag got_first_scan_ so that all value(particles) will be redefined in function laserCallback_s();

  std_msgs::Int32 condition;
  condition.data = CONDITION_INITICAL;
  slam_condition_pub.publish(condition);

  //build a new value to copy the initilization map
  nav_msgs::OccupancyGrid map_in_s;
  map_in_s = map_pose_in->map;
  //init map;
  initMap(map_in_s);

  //build a new value to copy the initilization positon
  geometry_msgs::Pose pose_in_s;
  pose_in_s = map_pose_in->pose;
  //get laser pose form base_link pose
  tf::Transform transform_map2base;
  pose2tansform(pose_in_s, transform_map2base);
  tf::Transform transform_map2laser = transform_map2base * transform_base2laser;
  tansform2pose(transform_map2laser, pose_in_s);
  //init pose;
  initPose(pose_in_s);

  //refresh all the setting by using funtion laserCallback_s(Class& scan);
  if(got_first_scan_){
        is_the_firstSacn_after_init_map = true;
  	got_first_scan_ = false;
	got_map_ = false;
	laserCallback_s(scan_last);
  }
}

void SlamGMapping::poseCovInitCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseCov_in)	//initicalize map using income map & income position
{
	if(!got_first_scan_ || !got_map_)
	{ return; }

	std_msgs::Int32 condition;
	condition.data = CONDITION_INITICAL;
	slam_condition_pub.publish(condition);

	ROS_INFO("poseCov_initial_in at time: %.2f", poseCov_in->header.stamp.toSec());

	//build a new value to copy the initilization positon
	geometry_msgs::Pose pose_in_s;
	pose_in_s.position = poseCov_in->pose.pose.position;
	pose_in_s.orientation = poseCov_in->pose.pose.orientation;
	//get laser pose form base_link pose
	tf::Transform transform_map2base;
	pose2tansform(pose_in_s, transform_map2base);
	tf::Transform transform_map2laser = transform_map2base * transform_base2laser;
	tansform2pose(transform_map2laser, pose_in_s);

	//build a new value to copy the initiization covariance
	Eigen::MatrixXd cov(6,6);
	for(int i=0; i< poseCov_in->pose.covariance.size(); i++)
	{ cov(i) = poseCov_in->pose.covariance[i]; }
	//initial initial_cov
	if(cov(0,0) < 2 && cov(0,0) > 0.01)
	{ initial_cov_xx_ = cov(0,0); }
	if(cov(1,1) < 2 && cov(1,1) > 0.01)
	{ initial_cov_yy_ = cov(1,1); }
	if(cov(3,3) < 2 && cov(3,3) > 0.01)
	{ initial_cov_aa_ = cov(3,3); }

	//init pose;
	initPose(pose_in_s);

	//refresh all the setting by using funtion laserCallback_s(Class& scan);
	//FIXME the old map in each particle will be deleted after refreshing. that is not what i want!!!
	if(got_first_scan_){
		is_the_firstSacn_after_init_map = true;
		got_first_scan_ = false;
		got_map_ = false;
		laserCallback_s(scan_last);
	}
}

void SlamGMapping::poseCovEKFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseCov_in)	//operate a pose proposal to update partice_pose using income pose
{
	if(!got_first_scan_ || !got_map_)
	{ return; }

	ROS_INFO("poseCov_ekf_in at time: %.2f", poseCov_in->header.stamp.toSec());

	//build a new value to copy the initilization positon
	geometry_msgs::Pose pose_in_s;
	pose_in_s.position = poseCov_in->pose.pose.position;
	pose_in_s.orientation = poseCov_in->pose.pose.orientation;
	//get laser pose form base_link pose
	tf::Transform transform_map2base;
	pose2tansform(pose_in_s, transform_map2base);
	tf::Transform transform_map2laser = transform_map2base * transform_base2laser;
	tansform2pose(transform_map2laser, pose_in_s);

	double x_ob, y_ob, z_ob;
	double roll_ob, pitch_ob, yaw_ob;
	geometry_msgs::Point position_ob = pose_in_s.position;
	x_ob = position_ob.x; 
	y_ob = position_ob.y;
	z_ob = position_ob.z;
	geometry_msgs::Quaternion orientation_ob = pose_in_s.orientation;
	tf::Quaternion q_ob(orientation_ob.x, orientation_ob.y, orientation_ob.z, orientation_ob.w);
	tf::Matrix3x3(q_ob).getRPY(roll_ob, pitch_ob, yaw_ob);

	//build a new value to copy the initiization covariance
	Eigen::MatrixXd cov(6,6);
	for(int i=0; i< poseCov_in->pose.covariance.size(); i++)
	{ cov(i) = poseCov_in->pose.covariance[i]; }
	//initial initial_cov
	double dxx, dyy, daa;
	dxx = sqrt(initial_cov_xx_);
	dyy = sqrt(initial_cov_yy_);
	daa = sqrt(initial_cov_aa_);
	if(cov(0,0) < 2 && cov(0,0) > 0.01)
	{ dxx = sqrt(cov(0,0)); }
	if(cov(1,1) < 2 && cov(1,1) > 0.01)
	{ dyy = sqrt(cov(1,1)); }
	if(cov(3,3) < 2 && cov(3,3) > 0.01)
	{ daa = sqrt(cov(3,3)); }

	for(int i=0; i< gsp_->m_particles.size(); i++)
	{
		double x = gsp_->m_particles[i].pose.x;
		double y = gsp_->m_particles[i].pose.y;
		double yaw = gsp_->m_particles[i].pose.theta;

		double dx = x_ob - x;
		double dy = y_ob - y;
		double dyaw = yaw_ob - yaw;
		dx = abs(dx); dy = abs(dy);dyaw = abs(dyaw);

		double dweight = (dx/dxx) * (dy/dyy);
		
		gsp_->m_particles[i].weight = gsp_->m_particles[i].weight * dweight;
	}

	double low_pass_h = 0.5;
	double low_pass_s = 1 - low_pass_h;
	for(int i=0; i< gsp_->m_particles.size(); i++)
	{
		gsp_->m_particles[i].pose.x 	= low_pass_h * gsp_->m_particles[i].pose.x 	+ low_pass_s * x_ob;
		gsp_->m_particles[i].pose.y 	= low_pass_h * gsp_->m_particles[i].pose.y 	+ low_pass_s * y_ob;
		gsp_->m_particles[i].pose.theta = low_pass_h * gsp_->m_particles[i].pose.theta  + low_pass_s * yaw_ob;
	}
}

void SlamGMapping::initPose(geometry_msgs::Pose& pose_in)
{
  //in this funtion:
  //1. reform the map form type geometry_msgs::Pose to GMapping_Fix::OrientedPoint
  //2. save the position into pose_init_
  //3. set the got_init_pose_ flag so that the slam will init map after next scan comming -> in function addScan().
  double x, y, z;
  double roll, pitch, yaw;
  geometry_msgs::Point position = pose_in.position;
  x = position.x; 
  y = position.y;
  z = position.z;
  geometry_msgs::Quaternion orientation = pose_in.orientation;
  tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  //save the pose into value of "GMapping_Fix::OrientedPoint pose_init_";
  pose_init_.x = x;
  pose_init_.y = y;
  pose_init_.theta = yaw;
  printf("pose_init: center : <%.2f, %.2f, %.2f> \t(meter)\n", x, y, yaw);

  //set the flag -> the real init of particle is in function addScan();
  got_init_pose_ = true;
}

void SlamGMapping::initMap(nav_msgs::OccupancyGrid& map_in)
{
  //in this funtion:
  //1. reform the map form type nav_msgs::OccupancyGrid to GMapping_Fix::ScanMatcherMap
  //2. save the map into smap_init_ (both white(empty area) and black(occupied area)) 
  //3. save the map into smap_init_occupied_ (black(occupied area) only)
  //4. set the got_init_map_ flag so that the slam will init map after next scan comming -> in function addScan().
  nav_msgs::OccupancyGrid map_in_s;
  map_in_s.header = map_in.header;
  map_in_s.info = map_in.info;
  map_in_s.data = map_in.data;

  //x -> width; y -> height
  int width_in = map_in_s.info.width;
  int height_in = map_in_s.info.height;
  double resolution = map_in_s.info.resolution;

  double roll, pitch, yaw;
  geometry_msgs::Point position = map_in_s.info.origin.position;
  double x = position.x;
  double y = position.y;
  double z = position.z;
  geometry_msgs::Quaternion orientation = map_in_s.info.origin.orientation;
  tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  //FIXME now it can handle a rotate map in this version!!!
  if(yaw != 0)
  { ROS_ERROR("can not init a rotate map in this version!!! with a yaw angle of %.2f degree.\nmap init fail", yaw * 180 / 3.14159); return; }
  
  double xmin_in = x;				//meter
  double xmax_in = x + width_in * resolution;	//meter
  double ymin_in = y;				//meter
  double ymax_in = y + height_in * resolution;	//meter

  xmin_ = xmin_in;
  xmax_ = xmax_in;
  ymin_ = ymin_in;
  ymax_ = ymax_in;
  delta_ = resolution; ///??should i change the resolution? if the parameter is different?

  GMapping_Fix::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  printf("map_init: center : <%.2f, %.2f> \t(meter)\n", center.x, center.x);
  printf("map_init: size   : <%d, %d> \t(pixel)\n", (int)width_in, (int)height_in);
  printf("map_init: area   : [%.2f, %.2f] -> [%.2f, %.2f] \t(meter)\n", xmin_, ymin_, xmax_, ymax_);
  printf("map_init: resolut: %.2f \t(meter)\n", delta_);

  GMapping_Fix::ScanMatcher matcher;  
  //a fake gsp_sensor
  GMapping_Fix::RangeSensor* gsp_laser_fake;
  unsigned int gsp_laser_beam_count_fake = 1;
  unsigned int scan_angle_increment_fake = 0;
  GMapping_Fix::OrientedPoint gmap_pose_fake(0, 0, 0);
  double maxRange_fake = 100.0;
  double maxUrange_fake = maxRange_fake;
  int scan_size = gsp_laser_beam_count_fake; //scan.ranges.size();
  double scan_data[1] = {0};
  gsp_laser_fake = new GMapping_Fix::RangeSensor("FLASER",
                                         gsp_laser_beam_count_fake,
                                         scan_angle_increment_fake,
                                         gmap_pose_fake,
                                         0.0,
                                         maxRange_fake);
  matcher.setLaserParameters(scan_size, scan_data, gsp_laser_fake->getPose());
  matcher.setlaserMaxRange(maxRange_fake);
  matcher.setusableRange(maxUrange_fake);
  matcher.setgenerateMap(true);

  GMapping_Fix::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);
  GMapping_Fix::ScanMatcherMap smap_occupied(center, xmin_, ymin_, xmax_, ymax_, delta_);
  int center_x_pixel = smap.getMapSizeX() / 2;
  int center_y_pixel = smap.getMapSizeY() / 2;
  //printf("ScanMatcherMap size: <%.2f,%.2f>\n", smap.getWorldSizeX(), smap.getWorldSizeY());

  //save the input map into smap_init_ & smap_init_occupied_ which is used to reiniticailize the system
  //smap_init_ is with empty area. smap_init_occupied_ is used for icp for slam, it doesnot has the empty part
  for(int x_cell=0; x_cell< smap.getMapSizeX(); x_cell++)	//x_cell : pixels
  {
    for(int y_cell=0; y_cell< smap.getMapSizeY(); y_cell++)
    {
      char data = map_in_s.data[MAP_IDX(map_in_s.info.width, x_cell, y_cell)];
      /// possibility relation: (char)map[0,100] -> (double)occ[0,1];    map -1 -> occ -1 unknown
      double occ = data * 0.01;
      bool is_occupied = false; //ture = black; false = gray
      if(occ < 0 || occ > 1.00000001)	//unknow is data is out of rang
      { occ = -1; continue; }
      else if( occ > occ_thresh_ )	// occupied if occ is bigger than thre-gate
      { occ = 1; is_occupied = true; }
      else if( occ < 0.2 )		// empty if occ is smaller than thre-gate
      { occ = 0; is_occupied = false; } 
      
      //get the position with meter
      GMapping_Fix::IntPoint p(x_cell, y_cell);
      double x_meter = (x_cell - center_x_pixel) * delta_;
      double y_meter = (y_cell - center_y_pixel) * delta_;

      // add data into possility map
      matcher.invalidateActiveArea();
      matcher.computeActivePoint(smap, x_meter, y_meter);
      matcher.registerPoint(smap, x_meter, y_meter, is_occupied);
      // add data into possility map which has occupied information only;
      if(is_occupied){
        matcher.invalidateActiveArea();
        matcher.computeActivePoint(smap_occupied, x_meter, y_meter);
        matcher.registerPoint(smap_occupied, x_meter, y_meter, is_occupied);
      }
    }
  }
  //save the map
  smap_init_ = new GMapping_Fix::ScanMatcherMap(smap);
  smap_init_occupied_ = new GMapping_Fix::ScanMatcherMap(smap_occupied);

  //set the flag  -> the real init of particle is in function addScan();
  got_init_map_ = true;
 
  //update the map to publish
  map_.map.info = map_in_s.info;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  /*get_mapformSmap(smap, map_.map);
  //get_mapformSmap(gsp_->m_particles[0].map, map_.map);
  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );
  //publish map
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  */
}

void SlamGMapping::get_mapformSmap(GMapping_Fix::ScanMatcherMap& smap, nav_msgs::OccupancyGrid& map) //reform the smap into ros msg
{
  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      GMapping_Fix::IntPoint p(x, y);
      double occ = smap.cell(p);
      assert(occ <= 1.0);

      if(occ < 0)
      { map.data[MAP_IDX(map.info.width, x, y)] = -1; }
      else if(occ > occ_thresh_)
      { map.data[MAP_IDX(map.info.width, x, y)] = 100; }
      else
      { map.data[MAP_IDX(map.info.width, x, y)] = 0; }
    }
  }
}

bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
  { return false; }
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

void SlamGMapping::publishParticleSample()
{
	//define header
	std_msgs::Header poseArray_header;
	poseArray_header.frame_id = "map";
	poseArray_header.stamp 	= ros::Time::now();
        poseArray_header.seq  	= 255;

	//define poseArray
	geometry_msgs::PoseArray poseArray;
	poseArray.header = poseArray_header;
	
	for(int i=0; i< gsp_->m_particles.size(); i++)
  	{
		//pick pose out of particle
		GMapping_Fix::OrientedPoint pose_particle = gsp_->m_particles[i].pose;
		double x = pose_particle.x;
		double y = pose_particle.y;
		double yaw = pose_particle.theta;

		geometry_msgs::Pose pose;
		pose.position.x = x;	//define position
		pose.position.y = y;
		pose.position.z = 0.0;
		tf::Quaternion q;	//define direction
		q.setRPY(0, 0, yaw); //q.setRPY(0, 0, 0);
		geometry_msgs::Quaternion q_msg;
		tf::quaternionTFToMsg(q, q_msg);
		pose.orientation = q_msg;

		poseArray.poses.push_back(pose);
  	}
	paticleSample_pub.publish(poseArray);
}

double d_Rand()
{
  return ( ((double)(rand()-RAND_MAX/2)) / (RAND_MAX/2) );
}

bool SlamGMapping::combine_smap(GMapping_Fix::ScanMatcherMap& smap_combined, const GMapping_Fix::ScanMatcherMap& smap_used)
{
  	GMapping_Fix::ScanMatcher matcher;  
  	//a fake gsp_sensor
  	GMapping_Fix::RangeSensor* gsp_laser_fake;
  	unsigned int gsp_laser_beam_count_fake = 1;
  	unsigned int scan_angle_increment_fake = 0;
  	GMapping_Fix::OrientedPoint gmap_pose_fake(0, 0, 0);
  	double maxRange_fake = 100.0;
  	double maxUrange_fake = maxRange_fake;
  	int scan_size = gsp_laser_beam_count_fake; //scan.ranges.size();
  	double scan_data[1] = {0};
  	gsp_laser_fake = new GMapping_Fix::RangeSensor("FLASER",
                                         gsp_laser_beam_count_fake,
                                         scan_angle_increment_fake,
                                         gmap_pose_fake,
                                         0.0,
                                         maxRange_fake);
  	matcher.setLaserParameters(scan_size, scan_data, gsp_laser_fake->getPose());
  	matcher.setlaserMaxRange(maxRange_fake);
  	matcher.setusableRange(maxUrange_fake);
  	matcher.setgenerateMap(true);

	if(smap_combined.getMapSizeX() != smap_used.getMapSizeX() || smap_combined.getMapSizeY() != smap_used.getMapSizeY())
	{ return false; } //FIXME if the map size is different, it shoube be resized!!!!!

	double reslution 	= smap_combined.getResolution();				//meter
	int center_x_pixel 	= smap_combined.getMapSizeX() / 2;				//pixel
	int center_y_pixel 	= smap_combined.getMapSizeY() / 2;				//pixel
	double x_min, x_max, y_min, y_max;
	smap_combined.getSize(x_min, y_min, x_max, y_max);
	
	for(int x_cell=0; x_cell< smap_used.getMapSizeX(); x_cell++)	//x_cell : pixels
	{
		for(int y_cell=0; y_cell< smap_used.getMapSizeY(); y_cell++)
		{
			GMapping_Fix::IntPoint p(x_cell, y_cell);
			double occ = smap_used.cell(p);

			if(occ < 0)
			{ continue; }
			bool is_occupied = false; //ture = black; false = gray
			if(occ < 0 || occ > 1.00000001)		//unknow is data is out of rang
			{ occ = -1; continue; }
			else if( occ > occ_thresh_ )	// occupied if occ is bigger than thre-gate
			{ occ = 1; is_occupied = true; }
			else if( occ < 0.2 )	// empty if occ is smaller than thre-gate
 			{ occ = 0; is_occupied = false; } 
			else
			{ continue; }

			//get the position with meter
			double x_meter = (x_cell - center_x_pixel) * reslution;
			double y_meter = (y_cell - center_y_pixel) * reslution;

			// add data into possility map
			matcher.invalidateActiveArea();
			matcher.computeActivePoint(smap_combined, x_meter, y_meter);
			matcher.registerPoint(smap_combined, x_meter, y_meter, is_occupied);
		}
      	}
}

void SlamGMapping::fakeScanUsingInitMap(sensor_msgs::LaserScan& 	scan_out, 
		const sensor_msgs::LaserScan& 		scan_in, 
		const GMapping_Fix::ScanMatcherMap& 	smap_init, 
		const GMapping_Fix::OrientedPoint& 	pose_init)
{
	int scan_size	= scan_in.ranges.size();
	double laser_pose_x 	= pose_init_.x;
	double laser_pose_y 	= pose_init_.y;
	double laser_pose_yaw 	= pose_init_.theta;

	double reslution = smap_init.getResolution();				//meter
	int center_x = smap_init.getMapSizeX() / 2;					//pixel
	int center_y = smap_init.getMapSizeY() / 2;					//pixel
	double x_min, x_max, y_min, y_max;
	smap_init.getSize(x_min, y_min, x_max, y_max);

	scan_out = scan_in;
	double angle_min 	= scan_out.angle_min;		//rad
	double angle_max 	= scan_out.angle_max;		//rad
	double angle_increment 	= scan_out.angle_increment;	//rad
	double range_min 	= minRange_ / 2;		//meter
	double range_max 	= maxRange_ * 2;		//meter
	double range_increment 	= reslution;			//meter
	for(int i=0; i<scan_size; i++)
	{
		double angle = angle_min + i*angle_increment;
		double direction = laser_pose_yaw + angle;
		double vec_x = cos(direction);
		double vec_y = sin(direction);
		double range_select = range_max;
		for(double range = range_min; range <= range_max; range += range_increment)
		{
			double x = vec_x * range + laser_pose_x;
			double y = vec_y * range + laser_pose_y;
			if(x>x_max || x<x_min || y>y_max || y<y_min)
			{ range_select = range_max; break; }
			int x_pixel = (int)(x/reslution) + center_x;
			int y_pixel = (int)(y/reslution) + center_y;
			GMapping_Fix::IntPoint p(x_pixel, y_pixel);
			double occ = smap_init.cell(p);
			if(occ < occ_thresh_ || occ > 1)
			{ continue; }
			else
			{ range_select = range; break; }
		}
		if(range_select < 0.7)
		{ range_select = range_max; }
		scan_out.ranges[i] = range_select;
	}
}

int SlamGMapping::read_map_formFile(const std::string map_yamlFileName, nav_msgs::OccupancyGrid& map, nav_msgs::MapMetaData& mapMetaData)
{
	//string map_pgmFileName = "2dmap.pgm";
	//string map_yamlFileName = "2dmap.yaml";

	std::string mapfname = "";   
	double origin[3];
	int negate;
	double occ_th, free_th;
	MapMode mode = TRINARY;
	std::string frame_id;

	std::ifstream fin(map_yamlFileName.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", map_yamlFileName.c_str());
          return 0; //exit(-1);
        }
	else
	{ printf("read map form file: %s,\n", map_yamlFileName.c_str()); }

	double res = resolution_;
	negate = negate_;
	occ_th = occupied_thresh_;
        free_th = free_thresh_; 
	origin[0] = origin_[0];
	origin[1] = origin_[1];
	origin[2] = origin_[2];
	mapfname = map_file_name_; //"/home/ojijin/catkin_ws/src/map_publisher_test/map/2dmap.pgm";
	/*change here!!! it shall have been read form .yaml file!*/

	//define map value
	nav_msgs::MapMetaData meta_data_message_;
	nav_msgs::GetMap::Response map_resp_;

	ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
	loadMapFromFile( &map_resp_, mapfname.c_str(), res, negate, occ_th, free_th, origin);
	map_resp_.map.info.map_load_time = ros::Time::now();
	map_resp_.map.header.frame_id = frame_id;
	map_resp_.map.header.stamp = ros::Time::now();
	ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
		map_resp_.map.info.width,
		map_resp_.map.info.height,
		map_resp_.map.info.resolution);
	meta_data_message_ = map_resp_.map.info;


	map = map_resp_.map;
	mapMetaData = meta_data_message_;

	return 1;
}

void pose2tansform(const geometry_msgs::Pose& pose_in, tf::Transform& transform_out)
{
	geometry_msgs::Point position		= pose_in.position;
	geometry_msgs::Quaternion orientation	= pose_in.orientation;

	double x, y, z;
	double roll, pitch, yaw;
	x = position.x;
	y = position.y;
	z = position.z;
	tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//printf("%.2f, %.2f, %.2f\t%.2f, %.2f, %.2f\n", x, y, z, roll, pitch, yaw);

	transform_out.setOrigin(tf::Vector3(x, y, z));
	//tf::Quaternion q;
	//q.setRPY(roll, pitch, yaw);
	transform_out.setRotation(q);
}

void tansform2pose(const tf::Transform& transform_in, geometry_msgs::Pose& pose_out)
{
	//double roll, pitch, yaw;
	//double x, y, z;
	//tf::Matrix3x3(transform_in.getRotation()).getRPY(roll, pitch, yaw);
	//tf::Vector3 position_vector =  transform_in.getOrigin();
	//x = position_vector.x();
	//y = position_vector.y();
	//z = position_vector.z();
	geometry_msgs::Transform geometry_Transform_msg;
	tf::transformTFToMsg(transform_in, geometry_Transform_msg);

	pose_out.position.x = geometry_Transform_msg.translation.x;
	pose_out.position.y = geometry_Transform_msg.translation.y;
	pose_out.position.z = geometry_Transform_msg.translation.z;
	pose_out.orientation.w = geometry_Transform_msg.rotation.w;
	pose_out.orientation.x = geometry_Transform_msg.rotation.x;
	pose_out.orientation.y = geometry_Transform_msg.rotation.y;
	pose_out.orientation.z = geometry_Transform_msg.rotation.z;
}
