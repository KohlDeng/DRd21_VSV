#ifndef PSI_H
#define PSI_H

#include <ros/ros.h>
#include <fsd_msgs/Map.h>
#include <string>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <vector>


class PSI {
	public:
		PSI(ros::NodeHandle* node_handle);

		void fusion_cloud_callback(const sensor_msgs::PointCloud2 &msg);
		void generate_map();
		void send_msg();
		fsd_msgs::Map get_map();

	private:
		ros::NodeHandle* nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		sensor_msgs::PointCloud2 fusion_cloud_in_;
		pcl::PointCloud<pcl::PointXYZL> fusion_cloud_pcl_;
		fsd_msgs::Map map_;
};


#endif