#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <std_msgs/Float64.h>

#include <lcm/lcm-cpp.hpp>
#include "lcm_to_ros/gga_t.h"
#include "lcm_to_ros/hyundai_mission.h"
#include "eurecar/gga_t.hpp"
#include "eurecar/hyundai_mission.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
//  <node pkg="rosbag" type="play" name="player" output="screen" args="--clock /home/ryu/catkin_ws/src/correction/bag/ref.bag"/>

//MAKE_REF :
//needs : rosbag file, lcm log file,
//do : rosbag recorder -O ref /correction/ref_center /correction/alpha

//DO_ICP:
//needs : ref rosbag

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define MAKE_REF 0
#define DO_ICP 1

#define PI 3.141592
class Correction
{
public:
	Correction();
	void refCallback(const PointCloud::ConstPtr& cloud_icp_ref);
	void alphaCallback(const std_msgs::Float64& alpha_ref);
	void lidarCallback(const PointCloud::ConstPtr& cloud);
//	void pathCallback(const lcm_to_ros::gga_t& path);
	void headingCallback(const lcm_to_ros::hyundai_mission& i30);
	void icp_lo(const PointCloud::Ptr& cloud_cur);
	void extract(const PointCloud::ConstPtr& cloud);
	void get_ref(const PointCloud::Ptr& cloud_filtered);

private:
	ros::NodeHandle nh_;
	ros::Subscriber Velodyne_scan;
	ros::Subscriber ref_point;
//	ros::Subscriber path_sub;
	ros::Subscriber heading;
	ros::Subscriber alpha_r;
	ros::Publisher filtered_cloud;
	ros::Publisher ref_center_cloud;
	ros::Publisher cur_center_cloud;
	ros::Publisher cloud_align_;
	ros::Publisher ref_cloud;
	ros::Publisher alpha_pub;

	PointCloud cloud;
	PointCloud cloud_c;

	PointCloud::Ptr cloud_filtered;
	PointCloud::Ptr cloud_cur;
	PointCloud::Ptr cloud_align;
	PointCloud::Ptr cloud_target;
	PointCloud::Ptr cloud_ref;
	PointCloud::Ptr cloud_icp_ref;

//	lcm_to_ros::gga_t::Ptr path;
	lcm_to_ros::hyundai_mission::Ptr i30;

	int distance_thershold_;
	int iteration_thershold_;
	int refer;
	int chk_once;
	float long_tm;
	float alpha_ref;
	float alpha_ref_;
	float beta;
	float error_long;
};

Correction::Correction()
{
	nh_ = ros::NodeHandle("~");

	filtered_cloud = nh_.advertise<PointCloud>("filtered_cloud",1);
	cur_center_cloud = nh_.advertise<PointCloud>("cur_center",1);
	cloud_align_ = nh_.advertise<PointCloud>("cloud_align",10);

	ref_cloud = nh_.advertise<PointCloud>("target",1);
	Velodyne_scan = nh_.subscribe("/merged_velodyne", 10,
																	&Correction::lidarCallback, this,
																	ros::TransportHints().tcpNoDelay(true));

#if MAKE_REF==1
	ref_center_cloud = nh_.advertise<PointCloud>("ref_center",1);
	alpha_pub = nh_.advertise<std_msgs::Float64>("alpha",1);
	heading = nh_.subscribe("/lcm_to_ros/hyundai_l2r", 1, &Correction::headingCallback, this);
#endif

#if DO_ICP==1
	ref_point = nh_.subscribe("/correction/ref_center", 10,
																	&Correction::refCallback, this,
																ros::TransportHints().tcpNoDelay(true));
	alpha_r = nh_.subscribe("/correction/alpha", 1, &Correction::alphaCallback, this);
#endif


  cloud_filtered = PointCloud::Ptr(new PointCloud);
	cloud_ref = PointCloud::Ptr(new PointCloud);
	cloud_cur = PointCloud::Ptr(new PointCloud);
	cloud_align = PointCloud::Ptr(new PointCloud);
	cloud_icp_ref = PointCloud::Ptr(new PointCloud);
	cloud_target = PointCloud::Ptr(new PointCloud);
	i30 = lcm_to_ros::hyundai_mission::Ptr (new lcm_to_ros::hyundai_mission);

	refer = 0;
	chk_once = 0;
	long_tm = 0.0;
	alpha_ref_ = 0.0;
	beta = 0.0;
	error_long = 0.0;
	nh_.param("disthershold", distance_thershold_, 10);
	nh_.param("iterthershold", iteration_thershold_ , 200);
}

void Correction::alphaCallback(const std_msgs::Float64& alpha_ref){
	alpha_ref_ = alpha_ref.data;
}


void Correction::headingCallback(const lcm_to_ros::hyundai_mission& i30)
{
	std_msgs::Float64 val;
	val.data = (double)i30.d_tmp1;
	alpha_pub.publish(val);
}

void Correction::refCallback(const PointCloud::ConstPtr& cloud_icp_ref)
{
	cloud_target->header.frame_id = "base_footprint";
	cloud_target->width = cloud_icp_ref->width;
	cloud_target->height = cloud_icp_ref->height;
	cloud_target->points.resize(cloud_target->width * cloud_target->height);

	if(chk_once == 0){
		for(int i = 0; i < cloud_icp_ref->points.size(); i++ ){
			cloud_target->points[i].x = cloud_icp_ref->points[i].x;
			cloud_target->points[i].y = cloud_icp_ref->points[i].y;
			cloud_target->points[i].z = cloud_icp_ref->points[i].z ;
		}
		chk_once++;
		cout << "ref" << endl;
	}
	ref_cloud.publish(*cloud_target);
}

void Correction::lidarCallback(const PointCloud::ConstPtr& cloud)
{
	extract(cloud);

#if MAKE_REF==1
	if(refer == 100){
		get_ref(cloud_filtered);
		refer++;
		cout << "count" << refer << endl;
	}else if(refer < 100){
		refer++;
	}
#endif

#if DO_ICP==1
	icp_lo(cloud_cur);

	alpha_ref_ = (27.6 * (PI/180)) - alpha_ref_;
	error_long = long_tm * cos(alpha_ref_);
	ROS_INFO("after : %f", error_long);
#endif
}

void Correction::icp_lo(const PointCloud::Ptr& cloud_cur)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setInputSource(cloud_cur);
	icp.setInputTarget(cloud_target);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (distance_thershold_);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (iteration_thershold_);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-9);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);
	icp.setRANSACOutlierRejectionThreshold (1.5);
	// Perform the alignment
	icp.align (*cloud_align);

	ROS_INFO("------------------------------");

	ROS_INFO("score : %f\n",icp.getFitnessScore());
	if (icp.hasConverged() && icp.getFitnessScore() <= 100){
		Eigen::Matrix4f transformation = icp.getFinalTransformation();
		long_tm = transformation(0,3)*(-1);
		ROS_INFO("before : %f",long_tm);
	} else{
		long_tm = 0.0;
		ROS_INFO("before : %f",long_tm);
	}
	cur_center_cloud.publish(*cloud_cur);
	cloud_align_.publish(*cloud_align);
}

void Correction::get_ref(const PointCloud::Ptr& cloud_filtered)
{

	cloud_ref->header.frame_id = "base_footprint";
	cloud_ref->width = cloud_filtered->width;
	cloud_ref->height = cloud_filtered->height;
	cloud_ref->points.resize(cloud_ref->width * cloud_ref->height);

	for(int i = 0; i < cloud_icp_ref->points.size(); i++ ){
		cloud_ref->points[i].x = cloud_filtered->points[i].x;
		cloud_ref->points[i].y = cloud_filtered->points[i].y;
		cloud_ref->points[i].z = cloud_filtered->points[i].z ;
	}

	*cloud_ref = PointCloud(*cloud_filtered);
	ref_center_cloud.publish(*cloud_ref);
}

void Correction::extract(const PointCloud::ConstPtr& cloud)
{
	//filtering
	pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud (cloud);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (-50.0, 70.0);
  //pass.setFilterLimitsNegative (true);
  passx.filter (*cloud_filtered);

	pcl::PassThrough<pcl::PointXYZ> passz;
	passz.setInputCloud (cloud_filtered);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (4.5, 5.5);
	//pass.setFilterLimitsNegative (true);
	passz.filter (*cloud_filtered);

	*cloud_cur = PointCloud(*cloud_filtered);
	filtered_cloud.publish(*cloud_filtered);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "correction");
	Correction Correction;
	ros::spin();

	return 0;
}
