#include "perception_slam_interface/psi.h"

PSI::PSI(ros::NodeHandle* node_handle) {
	nh_ = node_handle;
	sub_ = nh_->subscribe("/fusion_ros/fusion_cloud", 10, &PSI::fusion_cloud_callback, this);
	pub_ = nh_->advertise<fsd_msgs::Map>("/perception/cones", 10);
}

void PSI::fusion_cloud_callback(const sensor_msgs::PointCloud2 &msg) {
	fusion_cloud_in_ = msg;
	pcl::fromROSMsg(fusion_cloud_in_, fusion_cloud_pcl_);
	generate_map();
	send_msg();
}

void PSI::generate_map() {
	geometry_msgs::Point temp;
	temp.z = 0;
	for (int i; i < fusion_cloud_pcl_.points.size(); i++) {
		switch (fusion_cloud_pcl_.points[i].label) {
			// label = 0 -> high yellow
			case 0 : {
				temp.x = fusion_cloud_pcl_.points[i].x;
				temp.y = fusion_cloud_pcl_.points[i].y;
				map_.cone_high_yellow.push_back(temp);
				break;
			}
			// label = 1 -> red
			case 1 : {
				temp.x = fusion_cloud_pcl_.points[i].x;
				temp.y = fusion_cloud_pcl_.points[i].y;
				map_.cone_red.push_back(temp);
				break;
			}
			// label = 2 -> blue
			case 2 : {
				temp.x = fusion_cloud_pcl_.points[i].x;
				temp.y = fusion_cloud_pcl_.points[i].y;
				map_.cone_blue.push_back(temp);
				break;
			}
			// label = 3 -> low yellow
			case 3 : {
				temp.x = fusion_cloud_pcl_.points[i].x;
				temp.y = fusion_cloud_pcl_.points[i].y;
				map_.cone_low_yellow.push_back(temp);
				break;
			}
		}
	}
}

fsd_msgs::Map PSI::get_map() { return map_;}

void PSI::send_msg() {
	pub_.publish(get_map());
}
