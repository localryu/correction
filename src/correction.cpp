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


//#include "velodyne_height_map/heightmap.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define REF 1;

class Correction
{
public:
	Correction();
	void lidarCallback(const PointCloud::ConstPtr& cloud);
	void icp_lo(const PointCloud::Ptr& cloud);
	void extract(const PointCloud::ConstPtr& cloud);
	void get_ref();


private:
	ros::NodeHandle nh_;
	PointCloud cloud;
	PointCloud cloud_c;
	PointCloud::Ptr cloud_filtered;
	int seg[10][50] = {{0,}};
	double threshold = 5;
	float mean_x[10] = {0.0,};
	float mean_y[10] = {0.0,};

	ros::Subscriber Velodyne_scan;
  	ros::Publisher filtered_cloud;
	ros::Publisher ref_center_cloud;
	ros::Publisher cur_center_cloud;
	
	PointCloud cloud;
	PointCloud cloud_c;
	PointCloud::Ptr cloud_filtered;
	PointCloud::Ptr cloud_ref;
	PointCloud::Ptr cloud_cur;

	int seg[10][50] = {{0,}};
	double threshold;
	float center_o[10][3] = {{0.0,}};
	float center[10][3] = {{0.0,}};
	float ref[10][3] = {{0.0,}};
	int refer;
	int num;

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
	
	cloud_filtered = PointCloud::Ptr(new PointCloud);
	cloud_ref = PointCloud::Ptr(new PointCloud);
	cloud_cur = PointCloud::Ptr(new PointCloud);
	threshold = 5;
	refer = 0;
	num = 0;

}

void Correction::lidarCallback(const PointCloud::ConstPtr& cloud)
{

	num = 0;
	extract(cloud);

#ifdef REF
	if(refer == 100){
		get_ref();
		refer++;
	}else if(refer < 100){
		refer++;
	}
	//cout << "count" << refer << endl;
#endif

	icp_lo(cloud_cur);


}


void Correction::icp_lo(const PointCloud::ConstPtr& cloud)
{
	cur_center_cloud.publish(*cloud_cur);

}

void Correction::get_ref(){

	int cloud_width = 0;
	for(int i = 0; i < 10; i++){
		ref[i][0] = center_o[i][0];
		ref[i][1] = center_o[i][1];
		ref[i][2] = center_o[i][2];
		if(center_o[i][0] != 0){
			cloud_width++;
		}
	}
	cloud_ref->header.frame_id = "base_footprint";
	cloud_ref->width = cloud_width;
	cloud_ref->height = 1;
	cloud_ref->points.resize(cloud_ref->width * cloud_ref->height);

	for (size_t j = 0; j < cloud_ref->points.size(); ++j){
		cloud_ref->points[j].x = ref[j][0];
		cloud_ref->points[j].y = ref[j][1];
		cloud_ref->points[j].z = ref[j][2];
	}
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
	cout << "PointCloud has: " << cloud_c.points.size () << " data points." << endl;

	int n = cloud_c.points.size ();
	for(int ind = 1; ind < n+1; ind++){
		q[ind] = ind;
	}

	for(int i = 0; i < cloud_c.points.size (); i++){
		int x = 0;
		chk = 0;
		for(int j = 0; j < cloud_c.points.size (); j++){
			d = pow((pow((cloud_c.points[i].x - cloud_c.points[j].x),2.0) + pow((cloud_c.points[i].y - cloud_c.points[j].y),2.0) + pow((cloud_c.points[i].z - cloud_c.points[j].z),2.0)),1.0/3.0);
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
		int val = 0;
		float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
		center_o[l][0] = 0.0;
		center_o[l][1] = 0.0;
		center_o[l][2] = 0.0;
		if(seg[l][0] != 0){
		num++;
		}
		for(int h = 0; h < cn[l]; h++){
			val = seg[l][h];
			sum_x = sum_x + cloud_c.points[val].x;
			sum_y = sum_y + cloud_c.points[val].y;
			sum_z = sum_z + cloud_c.points[val].z;
		}
		center_o[l][0] = sum_x/cn[l];
		center_o[l][1] = sum_y/cn[l];
		center_o[l][2] = sum_z/cn[l];
		//cout << "seg : " << l << "x : "<< center_o[l][0] << "y : " << center_o[l][1] << endl;
	}

	cloud_cur->header.frame_id = "base_footprint";
	cloud_cur->width = num;
	cloud_cur->height = 1;
	cloud_cur->points.resize(cloud_cur->width * cloud_cur->height);
	for (size_t j = 0; j < cloud_cur->points.size(); ++j){
		cloud_cur->points[j].x = center_o[j][0];
		cloud_cur->points[j].y = center_o[j][1];
		cloud_cur->points[j].z = center_o[j][2];
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "correction");
	Correction Correction;
	ros::spin();

	return 0;
}
