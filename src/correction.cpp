#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>


#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

//#include "velodyne_height_map/heightmap.h"
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define MAKE_REF 0
#define DO_ICP 1

class Correction
{
public:
	Correction();
	void lidarCallback(const PointCloud::ConstPtr& cloud);
	void icp_lo(const PointCloud::Ptr& cloud_cur);
	//void icp_lo(const PointCloud::ConstPtr& cloud_icp_ref);
	void extract(const PointCloud::ConstPtr& cloud);
	void refCallback(const PointCloud::ConstPtr& cloud_icp_ref);
	void get_ref(const PointCloud::Ptr& cloud_filtered);

private:
	ros::NodeHandle nh_;
	ros::Subscriber Velodyne_scan;
	ros::Subscriber ref_point;
	ros::Publisher filtered_cloud;
	ros::Publisher ref_center_cloud;
	ros::Publisher cur_center_cloud;
	ros::Publisher ref_cloud;

	PointCloud cloud;
	PointCloud cloud_c;

	PointCloud::Ptr cloud_filtered;
	PointCloud::Ptr cloud_cur;
	PointCloud::Ptr cloud_align;
	PointCloud::Ptr cloud_target;
	PointCloud::Ptr cloud_ref;
	PointCloud::Ptr cloud_icp_ref;




	int refer;
	int chk_once;
	float long_t;

};


Correction::Correction()
{
	nh_ = ros::NodeHandle("~");

	filtered_cloud = nh_.advertise<PointCloud>("filtered_cloud",1);
	cur_center_cloud = nh_.advertise<PointCloud>("cur_center",1);
	ref_cloud = nh_.advertise<PointCloud>("target",1);
#if MAKE_REF==1
	ref_center_cloud = nh_.advertise<PointCloud>("ref_center",1);
#endif
#if DO_ICP==1
	ref_point = nh_.subscribe("/correction/ref_center", 10,
																	&Correction::refCallback, this,
																ros::TransportHints().tcpNoDelay(true));
#endif
	Velodyne_scan = nh_.subscribe("/merged_velodyne", 10,
                                  &Correction::lidarCallback, this,
                                  ros::TransportHints().tcpNoDelay(true));

  cloud_filtered = PointCloud::Ptr(new PointCloud);
	cloud_ref = PointCloud::Ptr(new PointCloud);
	cloud_cur = PointCloud::Ptr(new PointCloud);
	cloud_align = PointCloud::Ptr(new PointCloud);
	cloud_icp_ref = PointCloud::Ptr(new PointCloud);
	cloud_target = PointCloud::Ptr(new PointCloud);

	refer = 0;
	chk_once = 0;
	long_t = 0.0;

}

void Correction::refCallback(const PointCloud::ConstPtr& cloud_icp_ref){
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
		//*cloud_target = PointCloud(*cloud_icp_ref);
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
#endif
}

void Correction::icp_lo(const PointCloud::Ptr& cloud_cur)
{

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setInputSource(cloud_cur);
	icp.setInputTarget(cloud_target);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (10);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (200);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);
	// Perform the alignment
	icp.align (*cloud_align);
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	long_t = transformation(1,3)*(-1); 
	cout << long_t<< endl;

	cur_center_cloud.publish(*cloud_cur);

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
	pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (4.3, 5.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

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
