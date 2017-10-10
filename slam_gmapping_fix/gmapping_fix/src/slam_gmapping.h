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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
using Eigen::MatrixXd;
using namespace Eigen; 

#include "gmapping_fix/gridfastslam/gridslamprocessor.h"
#include "gmapping_fix/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>
#include "gmapping_fix/gmapping_initMapPoseMsg.h"

//(0:none;1:init;2:working)
#define  CONDITION_NONE		0
#define  CONDITION_INITICAL	1
#define  CONDITION_WORKING	2

class SlamGMapping
{
  public:
    SlamGMapping();
    SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);
    ~SlamGMapping();

    void init();
    void startLiveSlam();
    void startReplay(const std::string & bag_fname, std::string scan_topic);
    void publishTransform();
  
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);	//update slam when it recieve map msg
    void laserCallback_s(sensor_msgs::LaserScan& scan);			//a copy of update slam when it recieve map msg
    void mapInitCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_in);	//initicalize map using income map
    void poseInitCallback(const geometry_msgs::Pose::ConstPtr& pose_in);	//initicalize map using income pose
    void mapPoseInitCallback(const gmapping_fix::gmapping_initMapPoseMsg::ConstPtr& map_pose_in);	//initicalize map&partice_pose using income map
    void poseCovInitCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseCov_in);	//initicalize partice_pose using income pose
    void poseCovEKFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseCov_in);	//operate a pose proposal to update partice_pose using income pose
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);		//map service
    void publishLoop(double transform_publish_period);

    void initMap(nav_msgs::OccupancyGrid& map_in);
    void initPose(geometry_msgs::Pose& pose_in);
    void get_mapformSmap(GMapping_Fix::ScanMatcherMap& smap, nav_msgs::OccupancyGrid& map);
    void publishParticleSample();
    bool combine_smap(GMapping_Fix::ScanMatcherMap& smap_combined, const GMapping_Fix::ScanMatcherMap& smap_used);
    void fakeScanUsingInitMap(sensor_msgs::LaserScan& 	scan_out, 
		const sensor_msgs::LaserScan& 		scan_in, 
		const GMapping_Fix::ScanMatcherMap& 	smap_init, 
		const GMapping_Fix::OrientedPoint& 	pose_init);
    int read_map_formFile(const std::string map_yamlFileName, nav_msgs::OccupancyGrid& map, nav_msgs::MapMetaData& mapMetaData);

//  publish:
    ros::NodeHandle node_;
    ros::Publisher entropy_publisher_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::Publisher slam_condition_pub;			//publish this slam system condition (0:none;1:init;2:working)
    ros::Publisher paticleSample_pub;			//publish all partile sample as the poseArray
    ros::Subscriber chatter_mapInitCallback_sub;	//read the map init info; with mapInitCallback()
    ros::Subscriber chatter_poseInitCallback_sub;	//read the partice_pose init info; with poseInitCallback()
    ros::Subscriber chatter_mapPoseInitCallback_sub;	//read the map&partice_pose init info; with mapPoseInitCallback()
    ros::Subscriber chatter_poseCovInitCallback_sub;	//read the poseCov information form qr or other localization system to init; with poseCovInitCallback()
    ros::Subscriber chatter_poseCovEKFCallback_sub;	//read the poseCov information form qr or other localization system to ekf; with poseCovEKFCallback()

    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;
    tf::StampedTransform transform_base2laser;		//a transform form base_link to laser; the particle pose is the laser(sensor) pose, we should get base_link(robot) pose

    GMapping_Fix::GridSlamProcessor* gsp_;
    GMapping_Fix::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    tf::Stamped<tf::Pose> centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping_Fix::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    bool got_init_map_;
    bool got_init_pose_;
    bool is_init_pose_param_;
    GMapping_Fix::OrientedPoint pose_init_;
    GMapping_Fix::ScanMatcherMap* smap_init_;
    GMapping_Fix::ScanMatcherMap* smap_init_occupied_;
    bool is_the_firstSacn_after_init_map;
    bool is_the_secondScan_after_init_map;

    //FIXME the class "GMapping_Fix::ScanMatcherMap" is not possible to call, why?
    //GMapping_Fix::ScanMatcherMap smap_x;
    sensor_msgs::LaserScan scan_last;

    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    boost::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping_Fix::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(sensor_msgs::LaserScan& scan, GMapping_Fix::OrientedPoint& gmap_pose);
    double computePoseEntropy();
    
    // Parameters used by GMapping
    double maxRange_;
    double minRange_;	//min range to update map in case the laser can not see closer object
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_a_;
    double initial_cov_xx_;
    double initial_cov_yy_;
    double initial_cov_aa_;
    std::string 	initial_map_path_;
    std::string 	image_; 			//"2dmap.pgm"
    double 		resolution_; 			//0.050000
    std::vector<double> origin_;			// [-100.000000, -100.000000, 0.000000]
    int 		negate_; 			//0
    double 		occupied_thresh_; 		//0.65
    double 		free_thresh_; 			//0.196
    std::string 	yaml_file_name_;
    std::string 	map_file_name_;
    nav_msgs::OccupancyGrid 	init_map_param_;
    nav_msgs::MapMetaData 	init_mapMetaData_param_;
    
    
    ros::NodeHandle private_nh_;
    
    unsigned long int seed_;
    
    double transform_publish_period_;
    double tf_delay_;
};
