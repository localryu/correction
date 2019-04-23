#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


#include "velodyne_height_map/heightmap.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//typedef Correction_pointcloud::PointXYZIR VPoint;
//typedef pcl::PointCloud<VPoint> VPointCloud;


class Correction
{
public:
	Correction();
	void lidarCallback(const PointCloud::ConstPtr& cloud);
	void icp_lo(const PointCloud::ConstPtr& cloud);
	void filter(const PointCloud::ConstPtr& cloud);
	void extract(const PointCloud::ConstPtr& cloud);


private:
	ros::NodeHandle nh_;
//	PointCloud::ConstPtr cloud;
	PointCloud cloud;
	PointCloud cloud_c;
	PointCloud::Ptr cloud_filtered;
	int seg[10][50] = {{0,}};
	double threshold = 5;
	float mean_x[10] = {0.0,};
	float mean_y[10] = {0.0,};

	ros::Subscriber Velodyne_scan;
  ros::Publisher filtered_cloud;
	ros::Publisher pub;
};


Correction::Correction()
{
	nh_ = ros::NodeHandle("~");

	//pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
	pub = nh_.advertise<sensor_msgs::PointCloud2> ("output", 1);
	//filtered_scan = nh_.advertise<PointCloud>("filter_for_correction",1);
	filtered_cloud = nh_.advertise<PointCloud>("filtered_cloud",1);
	Velodyne_scan = nh_.subscribe("/merged_velodyne", 10,
                                  &Correction::lidarCallback, this,
                                  ros::TransportHints().tcpNoDelay(true));

}

void Correction::lidarCallback(const PointCloud::ConstPtr& cloud)
{

	extract(cloud);

}


void Correction::icp_lo(const PointCloud::ConstPtr& cloud)
{
	//filtered_scan.publish(cloud_filtered);

}
/*
void Correction::filter(const PointCloud::ConstPtr& cloud){
	PointCloud::Ptr cloud_filtered (new PointCloud);

	pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (4.5, 5.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

	filtered_cloud.publish(*cloud_filtered);
}*/

void Correction::extract(const PointCloud::ConstPtr& cloud)
{
	//filtering
	PointCloud::Ptr cloud_filtered (new PointCloud);

	pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (4.5, 5.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

	filtered_cloud.publish(*cloud_filtered);

	//clustering
	float d = 10;
	int k = -1;
	int q[100] = {0,};
	int cn[20] = {0,};
	int np = 0;
	int chk = 0;
	cloud_c = PointCloud(*cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustred (new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "PointCloud has: " << cloud_c.points.size () << " data points." << std::endl;


	int n = cloud_c.points.size ();
	for(int ind = 1; ind < n+1; ind++){
		q[ind] = ind;
	}

	for(int i = 0; i < cloud_c.points.size (); i++){
		int x = 0;
		chk = 0;
		for(int j = 0; j < cloud_c.points.size (); j++){
			d = sqrt(pow((cloud_c.points[i].x - cloud_c.points[j].x),2.0) + pow((cloud_c.points[i].y - cloud_c.points[j].y),2.0));
			if(d < threshold){
				if(q[i] != 0){
					q[i] = 0;
					chk++;
					if(chk == 1){
						k++;
					}
					seg[k][x] = i;
					x++;
				}
				if(q[j] != 0){
					q[j] = 0;
					chk++;
					if(chk == 1){
						k++;
					}
					seg[k][x] = j;
					x++;
				}
			}
		}
		if(x != 0){
			cn[np] = x;
			np++;
		}
	}

		for(int l = 0 ; l < 10; l++){
			int num , val = 0;
			float sum_x = 0.0, sum_y = 0.0;
			mean_x[l] = 0.0;
			mean_y[l] = 0.0;
			for(int h = 0; h < cn[l]; h++){
				val = seg[l][h];
				sum_x = sum_x + cloud_c.points[val].x;
				sum_y = sum_y + cloud_c.points[val].y;
			}
			mean_x[l] = sum_x/cn[l];
			mean_y[l] = sum_y/cn[l];
			cout << "seg : " << l << "x : "<< mean_x[l] << "y : " << mean_y[l] << endl;
		}

}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "correction");
	Correction Correction;
	ros::spin();

	return 0;
}
