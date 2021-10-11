/*
 * slam_mapping
 */

/* Author: HoGinhang */

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64.h"
#include "fsd_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/Cone.h"
#include "std_msgs/Int16.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include"Point.hpp"

struct MapPoints
{
    //points和label的索引是对应的
    std::vector<Mapping::Point> points;
    std::vector<int> label;
};

class SlamMapping
{
    private:
        ros::NodeHandle node_;
        ros::Publisher MapPublisher_; 
        ros::Publisher LocalMapPublisher_;
        ros::Publisher VehStatePublisher_;

        ros::ServiceServer MapServer_;     
        ros::Publisher lapCountPublisher_;   //发布循迹圈数
        
        //可视化路径, 可视化地图
        ros::Publisher VisualPathPublisher_;
        ros::Publisher Red_VisualMapPublisher_;
        ros::Publisher Blue_VisualMapPublisher_;
        ros::Publisher LocalMap_VisualMapPublisher_;
        ros::Publisher BaseMap_VisualMapPublisher_;

        ros::Subscriber PointDataSubscriber_;   //订阅一次聚类点的数据，消息类型是sensor_msgs::PointCloud

        //rawdata接受的是sensor_msgs::PointCloud2，要转化为pcl::PointCloud<pcl::PointXYZI>
        //随后在转化为自定义的二维的Mapping::Point
        //等于说在接受到消息的时候，在回调函数里要进行点云的数据类型转换
        
        /*用于直接订阅三个激光雷达数据
        ros::Subscriber rawDataSubscriber1_;    
        ros::Subscriber rawDataSubscriber2_;
        ros::Subscriber rawDataSubscriber3_;
        */
       std::string map_frame_;

        ros::Subscriber PathByRTK_;                 //用于订阅RTK的Path，系列的点
        ros::Subscriber SpeedByRTK_;
        ros::Subscriber AccByRTK_;

        ros::NodeHandle private_node_;      //用于处理自身参数初始化

        bool got_map_;
        bool got_calibrated_;
        bool got_clustered_;
        
        int laser_number_;    //激光数，表示激光数据在第几帧
        int odom_number_;    //odom数，表示odom数据在第几帧
        int lapCount_number_;    //lap计算帧数，表示每圈进来在第几帧

        void UpdateMap(const ros::Time& t, const  std::vector<fsd_common_msgs::Cone> &ScanCones, std::vector<fsd_common_msgs::Cone> &UpdateCones, std::vector<fsd_common_msgs::Cone> &HistoryCones);
        void getVehiclePose(Mapping::OrientedPoint& Vehicle_pose, const ros::Time& t);
        //void CalibrateMap(const int laser_number_, const ros::Time& t);

        int clusterOfMap(const std::vector<fsd_common_msgs::Cone> &CenterCones, const fsd_common_msgs::Cone &mUpdateCones);
        void getClusterPoints(std::vector<std::vector<fsd_common_msgs::Cone>> SortOutPoints, std::vector<fsd_common_msgs::Cone> &mClusterCone);

        void SecondCluster(std::vector<fsd_common_msgs::Cone> &mClusterCone, const std::vector<fsd_common_msgs::Cone> &CenterCones, std::vector<fsd_common_msgs::Cone> &UpdateCones,std::vector<fsd_common_msgs::Cone> &HistoryCones);

        void lapCount_Calculate(const Mapping::OrientedPoint &odom_pose);
        void generate_localmap(const Mapping::OrientedPoint &mpose);

        void Points_map_to_vehicle(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint);
        void Points_vehicle_to_map(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint);

    public:
        SlamMapping();
        SlamMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~SlamMapping(); /*析构函数*/

        void init(); 
        void startLiveSlam(); 
        //对bag进行建图，暂时放弃
        //void startReplay(const std::string & bag_fname, std::string scan_topic);
        
        void odomCallback(const geometry_msgs::Pose2D::ConstPtr& mpose);
        void SpeedCallback(const geometry_msgs::Vector3::ConstPtr& mspeed);
        void AccCallback(const geometry_msgs::Accel::ConstPtr& macc);

        void laserCallback(const fsd_common_msgs::Map::ConstPtr& laserCloudMsg);
        void Msg_to_Pub();
        double Caculate_Dist(const fsd_common_msgs::Cone &Cone1, const fsd_common_msgs::Cone &Cone2);
        void generate_CenterCone(std::vector<fsd_common_msgs::Cone> &mCenterCone, std::vector<fsd_common_msgs::Cone> &ScanCones);
        void Switch_to_Map(const std::vector<fsd_common_msgs::Cone> &ClusterCone_red, const std::vector<fsd_common_msgs::Cone> &ClusterCone_blue, const std::vector<fsd_common_msgs::Cone> &ClusterCone_yellow, const std::vector<fsd_common_msgs::Cone> &ClusterCone_unknow);
        void Visualization_Path();
        void Visualization_Map();
        void produce_localmap(std::vector<geometry_msgs::Point> &mlocalcones, std::vector<geometry_msgs::Point> &mapcones, Mapping::OrientedPoint &npose);

        //K表示簇数，是一个动态的变量
        int K;

        //车辆位姿
        std::vector<Mapping::OrientedPoint> VehiclePose;
        Mapping::OrientedPoint StartPose;
        std::vector<ros::Time> TimeSeries;
              
       //与Node通信有关
        fsd_msgs::Map map_;
        //fsd_msgs::Map basemap_;
        fsd_msgs::Map localmap_;
        fsd_common_msgs::CarState carstate_;
        std_msgs::Int16 lapCount_;
        nav_msgs::Path mpath_;

        //map上已扫描到的所有锥桶
        std::vector<fsd_common_msgs::Cone> HistoryUpdateCone_red;
        std::vector<fsd_common_msgs::Cone> HistoryUpdateCone_blue;
        std::vector<fsd_common_msgs::Cone> HistoryUpdateCone_yellow;
        std::vector<fsd_common_msgs::Cone> HistoryUpdateCone_unknow;

        //map上的聚类中心锥桶
        std::vector<fsd_common_msgs::Cone> CenterCone_red;
        std::vector<fsd_common_msgs::Cone> CenterCone_blue;
        std::vector<fsd_common_msgs::Cone> CenterCone_yellow;
        std::vector<fsd_common_msgs::Cone> CenterCone_unknow;


        //map上的二次聚类锥桶
        std::vector<fsd_common_msgs::Cone> ClusterCone_red;
        std::vector<fsd_common_msgs::Cone> ClusterCone_blue;
        std::vector<fsd_common_msgs::Cone> ClusterCone_yellow;
        std::vector<fsd_common_msgs::Cone> ClusterCone_unknow;

        visualization_msgs::Marker Map_RedCone;
        visualization_msgs::MarkerArray Map_RedConeArray;
        visualization_msgs::Marker Map_BlueCone;
        visualization_msgs::MarkerArray Map_BlueConeArray;

        visualization_msgs::Marker LocalMap_Cone;
        visualization_msgs::MarkerArray LocalMap_ConeArray;
        visualization_msgs::Marker BaseMap_Cone;
        visualization_msgs::MarkerArray BaseMap_ConeArray;
};

