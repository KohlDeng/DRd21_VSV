/*
 * slam_mapping
 */

/* Author: HoGinhang */

#include "Fortest_DemoSlamMapping.h"

#include <iostream>
#include <time.h>
#include <memory>

#include "ros/console.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/Cone.h"

//函数重载，不用管
SlamMapping::SlamMapping(){
    init();
}

/*析构函数，自动释放内存，不用管*/
SlamMapping::~SlamMapping(){}

/*SlamMapping的初始化，主要用来读取配置文件中写入的参数*/
void SlamMapping::init()
{
    got_map_ = false;
    got_calibrated_ = false;
    got_clustered_ = false;
    laser_number_ = 0;
    odom_number_=0;
    lapCount_number_=0;
    lapCount_.data = 0;
    map_frame_ = "map";
}

/*开始在线SLAM*/
void SlamMapping::startLiveSlam()
{
    //订阅一些主题，发布一些主题
    MapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/map", 1, true);
    LocalMapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/localmap", 1, true);
    VehStatePublisher_ = node_.advertise<fsd_common_msgs::CarState>("/slam/carstate",1,true);
    lapCountPublisher_ = node_.advertise<std_msgs::Int16>("/slam/lapcount",1,true);
    
    VisualPathPublisher_ = node_.advertise<nav_msgs::Path>("/slam/visualpath",1,true);
    Red_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualmapred",1,true);
    Blue_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualmapblue",1,true);
    LocalMap_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualocalmap",1,true);
    BaseMap_VisualMapPublisher_= node_.advertise<visualization_msgs::MarkerArray>("/slam/visuabasemap",1,true);

    //订阅RTK数据，消息类型未知，等于获得当前位姿
    PathByRTK_ = node_.subscribe("/veh_status/pose", 1, &SlamMapping::odomCallback, this);
    SpeedByRTK_= node_.subscribe("/veh_status/speed", 1, &SlamMapping::SpeedCallback, this);
    AccByRTK_= node_.subscribe("/veh_status/acc", 1, &SlamMapping::AccCallback, this);

    //订阅一次聚类点数据，消息类型未知
    PointDataSubscriber_ = node_.subscribe("/perception/ps_interface", 1, &SlamMapping::laserCallback, this);

    //打印消息
    std::cout <<"Subscribe PointCloudData & odom!!!"<<std::endl;

}

/*生成local发给控制器，理论上map_本身是按序排列的，只需要根据当前位置，发出前视多少个localmap就ok*/
void SlamMapping::generate_localmap(const Mapping::OrientedPoint &mpose){
    if(lapCount_.data == 0){
        ROS_INFO("generate_localmap default Successful");
        return;
    }

    Mapping::OrientedPoint npose = mpose;
    //第一圈直接返回，然后第二圈进来，肯定是有全局map_了
    localmap_.cone_high_yellow.clear();
    localmap_.cone_red.clear();
    localmap_.cone_blue.clear();
    localmap_.cone_low_yellow.clear();
    
    produce_localmap(localmap_.cone_blue, map_.cone_blue,npose);
    produce_localmap(localmap_.cone_red, map_.cone_red,npose);
}

 void SlamMapping::produce_localmap(std::vector<geometry_msgs::Point> &mlocalcones, std::vector<geometry_msgs::Point> &mapcones, Mapping::OrientedPoint &npose){
    //首先将全局地图，全部转换到base_link中，然后基于base_link中的点，找到前视的10个点，将其输出
    //基本原理是，判断点的纵向x是否大于0，然后是否为距离mpose最近的十个点，然后push到localmap_
    Mapping::Point mpoint;
    geometry_msgs::Point point_base_link;
    std::vector<geometry_msgs::Point> basemap_;
    double dist_min_plus = 999;
    double dist_base;
    //plus正向最近点、minor负向最近点
    int index_min_plus;
    bool index_min_notready = true;

    for(int i =0;i<mapcones.size();i++){
        mpoint.x = mapcones[i].x;
        mpoint.y = mapcones[i].y;
        Points_map_to_vehicle(npose, mpoint);
        point_base_link.x = mpoint.x;
        point_base_link.y = mpoint.y;
        //将当前map坐标系下的点全转到base_link下,索引与map_相同
        basemap_.push_back(point_base_link);

        //找当前车辆位置的对应点的索引
        if(mpoint.x >=0){
            dist_base = hypot(mpoint.x, mpoint.y);
            if(dist_base < dist_min_plus){
                dist_min_plus = dist_base;
                index_min_plus = i;
                index_min_notready = false;
            }
        }
    }
   ROS_INFO("after Loop Successful");

    if(index_min_notready){
        ROS_INFO("index_min_notready is true");
        return;
    }

    //首先将相关点塞进localmap
    mlocalcones.push_back(mapcones[index_min_plus]);

    ROS_INFO("after related Successful");
    //找到相关点，循环取当前相关点对应的邻近点
    int cone_range = 6; //表示localmap顺序传递6个点
    double x_now, y_now;
    double x_relate, y_relate;
    
    
    for(int j=0;j<cone_range - 1;j++){
        bool HavenotFoundNeighbor = true;
        for(int i =0;i<basemap_.size();i++){
            bool isTheSameSide = true;
            double dist_thershold = 5;
            x_now = basemap_[i].x;
            y_now = basemap_[i].y;
            x_relate = basemap_[index_min_plus].x;
            y_relate = basemap_[index_min_plus].y;

            if(HavenotFoundNeighbor && x_now > x_relate){
                if(hypot(x_now - x_relate, y_now - y_relate) <= dist_thershold){
                    index_min_plus = i;
                    HavenotFoundNeighbor = false;
                    mlocalcones.push_back(mapcones[index_min_plus]);
                }
            }
        }
    }
 }

/*将二次聚类点云转换到fsd_msgs::Map中*/
void SlamMapping::Switch_to_Map(const std::vector<fsd_common_msgs::Cone> &ClusterCone_red, const std::vector<fsd_common_msgs::Cone> &ClusterCone_blue, const std::vector<fsd_common_msgs::Cone> &ClusterCone_yellow, const std::vector<fsd_common_msgs::Cone> &ClusterCone_unknow){
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id =  map_frame_;
    
    //将ClusterCone的(x,y)放到map_里面去，然后发布出去
    geometry_msgs::Point point_in_map_;
    
    for(int i =0;i<ClusterCone_red.size();i++){
        point_in_map_.x = ClusterCone_red[i].position.x;
        point_in_map_.y = ClusterCone_red[i].position.y;
        point_in_map_.z = 0;
        map_.cone_red.push_back(point_in_map_);
    }
    for(int i =0;i<ClusterCone_blue.size();i++){
        point_in_map_.x = ClusterCone_blue[i].position.x;
        point_in_map_.y = ClusterCone_blue[i].position.y;
        point_in_map_.z = 0;
        map_.cone_blue.push_back(point_in_map_);
    }
    for(int i =0;i<ClusterCone_yellow.size();i++){
        point_in_map_.x = ClusterCone_yellow[i].position.x;
        point_in_map_.y = ClusterCone_yellow[i].position.y;
        point_in_map_.z = 0;
        map_.cone_low_yellow.push_back(point_in_map_);
    }
    //暂时认为unkonw与high_yellow对应一致
    for(int i =0;i<ClusterCone_unknow.size();i++){
        point_in_map_.x = ClusterCone_unknow[i].position.x;
        point_in_map_.y = ClusterCone_unknow[i].position.y;
        point_in_map_.z = 0;
        map_.cone_high_yellow.push_back(point_in_map_);
    }

    ROS_INFO("Quantity of Red Cone is %d", map_.cone_red.size());
    ROS_INFO("Quantity of Blue Cone is %d", map_.cone_blue.size());
        
}

/*将点从map转换到base_link，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_map_to_vehicle(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint){
    double vtheta = mpose.theta;
    double vx = mpose.x;
    double vy = mpose.y;
    double mx = mpoint.x;
    double my = mpoint.y;
    
    double deltax =mx - vx;
    double deltay = my - vy;

    mpoint.x = deltax * cos(vtheta) + deltay * sin(vtheta);
    mpoint.y = deltay * cos(vtheta) - deltax * sin(vtheta);
}

/*将点从base_link转换到map，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_vehicle_to_map(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint){
    Mapping::OrientedPoint Vehicle_pose = mpose;
    double mx = mpoint.x;
    double my = mpoint.y;
    mpoint.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) - my * sin(Vehicle_pose.theta);
    mpoint.y = Vehicle_pose.y + mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
}

/*里程计信息的回调函数*/
void SlamMapping::odomCallback(const geometry_msgs::Pose2D::ConstPtr& mpose)
{
    odom_number_ ++;

    Mapping::OrientedPoint odom_pose;
    odom_pose.x = mpose->x;
    odom_pose.y = mpose->y;
    odom_pose.theta = mpose->theta;
    VehiclePose.push_back(odom_pose);

    if(odom_number_ <= 40){
        StartPose.x = odom_pose.x;
        StartPose.y = odom_pose.y;
        StartPose.theta = odom_pose.theta;
    }

    lapCount_Calculate(odom_pose);
    ROS_INFO("lapCount_Calculate Successful");
    
    generate_localmap(odom_pose);
    ROS_INFO("No Wrong in generate_localmap");

    //position at Map
    carstate_.car_state.x = odom_pose.x;
    carstate_.car_state.y = odom_pose.y;
    carstate_.car_state.theta = odom_pose.theta;

    Visualization_Path();
    ROS_INFO("OdomCallback Successful");
}

void SlamMapping::SpeedCallback(const geometry_msgs::Vector3::ConstPtr& mspeed){
    //velocities at Map, 分别是纵向速度、横向速度和横摆角速度
    carstate_.car_state_dt.car_state_dt.x = mspeed->x;
    carstate_.car_state_dt.car_state_dt.y = mspeed->y;
    carstate_.car_state_dt.car_state_dt.theta = mspeed->z;
    ROS_INFO("SpeedCallback Successful");
}

void SlamMapping::AccCallback(const geometry_msgs::Accel::ConstPtr& macc){
    //accleration at Map
    carstate_.car_state_dt.car_state_a.x = macc->linear.x;
    carstate_.car_state_dt.car_state_a.y = macc->linear.y;
    carstate_.car_state_dt.car_state_a.theta = macc->angular.z;
    ROS_INFO("AccCallback Successful");
}

/*发布消息*/
void SlamMapping::Msg_to_Pub(){
    VehStatePublisher_.publish(carstate_);
    lapCountPublisher_.publish(lapCount_);
    MapPublisher_.publish(map_);
    LocalMapPublisher_.publish(localmap_);
    VisualPathPublisher_.publish(mpath_);
    Red_VisualMapPublisher_.publish(Map_RedConeArray);
    Blue_VisualMapPublisher_.publish(Map_BlueConeArray);
    LocalMap_VisualMapPublisher_.publish(LocalMap_ConeArray);
    BaseMap_VisualMapPublisher_.publish(BaseMap_ConeArray);
    
    ROS_INFO("Msg_to_Pub Successful");
}

void SlamMapping::Visualization_Path(){
    ros::Time now_stamp = ros::Time::now();
    geometry_msgs::PoseStamped mposestamp_;

    mpath_.header.stamp = now_stamp;
    mpath_.header.frame_id = "map";

    mposestamp_.header.stamp = now_stamp;
    mposestamp_.pose.position.x = carstate_.car_state.x;
    mposestamp_.pose.position.y = carstate_.car_state.y;
    mposestamp_.pose.position.z = 0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(carstate_.car_state.theta);
    mposestamp_.pose.orientation.x = goal_quat.x;
    mposestamp_.pose.orientation.y = goal_quat.y;
    mposestamp_.pose.orientation.z = goal_quat.z;
    mposestamp_.pose.orientation.w = goal_quat.w;

    mpath_.poses.push_back(mposestamp_);
}

void SlamMapping::Visualization_Map(){
    // visualization_msgs::MarkerArray Map_RedCones_;
    Map_RedConeArray.markers.clear();
    Map_BlueConeArray.markers.clear();

    //红色锥桶可视化
    Map_RedCone.header.frame_id = "map";
    Map_RedCone.header.stamp = ros::Time::now();
    Map_RedCone.ns = "red";
    Map_RedCone.action = visualization_msgs::Marker::ADD;
    Map_RedCone.type = visualization_msgs::Marker::SPHERE;
    Map_RedCone.scale.x = 0.3;
    Map_RedCone.scale.y = 0.3;
    Map_RedCone.scale.z = 0.3;
    Map_RedCone.color.a = 0.5;
    Map_RedCone.color.r = 255;
    Map_RedCone.color.g = 0;
    Map_RedCone.color.b = 0;
    Map_RedCone.color.a = 1.0;
    
    int i=0;
    for(const auto &iter: map_.cone_red) {
        Map_RedCone.pose.position.x = iter.x;
        Map_RedCone.pose.position.y = iter.y;
        Map_RedCone.id = i;
        i++;
        //Map_RedCone.points.push_back(p);
        Map_RedConeArray.markers.push_back(Map_RedCone);
        if(i>1000) break;
    }

    //蓝色锥桶可视化
    Map_BlueCone.header.frame_id = "map";
    Map_BlueCone.header.stamp = ros::Time::now();
    Map_BlueCone.ns = "blue";
    Map_BlueCone.action = visualization_msgs::Marker::ADD;
    Map_BlueCone.type = visualization_msgs::Marker::SPHERE;
    Map_BlueCone.scale.x = 0.3;
    Map_BlueCone.scale.y = 0.3;
    Map_BlueCone.scale.z = 0.3;
    Map_BlueCone.color.a = 0.5;
    Map_BlueCone.color.r = 0;
    Map_BlueCone.color.g = 0;
    Map_BlueCone.color.b = 255;
    Map_BlueCone.color.a = 1.0;
    
    int j=0;
    for(const auto &iter: map_.cone_blue) {
        Map_BlueCone.pose.position.x = iter.x;
        Map_BlueCone.pose.position.y = iter.y;
        Map_BlueCone.id = j;
        j++;
        Map_BlueConeArray.markers.push_back(Map_BlueCone);
        if(j>1000) break;
    }

    //LocalMap可视化
    LocalMap_ConeArray.markers.clear();
    LocalMap_Cone.header.frame_id = "map";
    LocalMap_Cone.header.stamp = ros::Time::now();
    LocalMap_Cone.ns = "green";
    LocalMap_Cone.action = visualization_msgs::Marker::ADD;
    LocalMap_Cone.type = visualization_msgs::Marker::SPHERE;
    LocalMap_Cone.scale.x = 0.5;
    LocalMap_Cone.scale.y = 0.5;
    LocalMap_Cone.scale.z = 0.5;
    LocalMap_Cone.color.a = 0.5;
    LocalMap_Cone.color.r = 0;
    LocalMap_Cone.color.g = 255;
    LocalMap_Cone.color.b = 0;
    LocalMap_Cone.color.a = 1.0;

    int k=0;
    for(const auto &iter: localmap_.cone_blue) {
        LocalMap_Cone.pose.position.x = iter.x;
        LocalMap_Cone.pose.position.y = iter.y;
        LocalMap_Cone.id = k;
        k++;
        LocalMap_ConeArray.markers.push_back(LocalMap_Cone);
        if(k>100) break;
    }
    for(const auto &iter: localmap_.cone_red) {
        LocalMap_Cone.pose.position.x = iter.x;
        LocalMap_Cone.pose.position.y = iter.y;
        LocalMap_Cone.id = k;
        k++;
        LocalMap_ConeArray.markers.push_back(LocalMap_Cone);
        if(k>100) break;
    }

    ROS_INFO("Visualization_Map Successful");
}

//lapCount用来识别车辆经过第几圈，具体策略是利用定位来确认
//odom是200Hz，Lidar是10Hz
void SlamMapping::lapCount_Calculate(const Mapping::OrientedPoint &odom_pose){
    //lapCount_number帧数小于1000的话，就不进行lapCount计算
    lapCount_number_ ++;
    int lapCount_thershold = 1000;
    if(lapCount_number_ <= lapCount_thershold){
        return;
    }

    Mapping::OrientedPoint base_pose;
    base_pose.x = StartPose.x;
    base_pose.y = StartPose.y;
    base_pose.theta = StartPose.theta;
    double dist_to_basepoint = euclidianDist(base_pose, odom_pose);
    //车辆与原点距离小于0.1m，则认为在原点附近
    double dist_thershold = 2;

    if(dist_to_basepoint <= dist_thershold){
        lapCount_.data++;
        ROS_INFO("now lapCount = %d", lapCount_.data);
        //车辆进过一圈后，lapCount_number_清0
        lapCount_number_ = 0;
    }
    
}

/*激光信息的回调函数，每次节点收到一条消息时都将调用此函数，这个函数也是主要函数*/
void SlamMapping::laserCallback(const fsd_common_msgs::Map::ConstPtr& laserCloudMsg)
{
   laser_number_++;
    int i=0;
    ros::Time now_stamp = ros::Time::now();
    ROS_INFO("Enter LaserCallback Successful at %d frame", laser_number_);

    //扫描到的base_link上的锥桶
    std::vector<fsd_common_msgs::Cone> ScanCones_red;
    std::vector<fsd_common_msgs::Cone> ScanCones_blue;
    std::vector<fsd_common_msgs::Cone> ScanCones_yellow;
    std::vector<fsd_common_msgs::Cone> ScanCones_unknow;

    //转化到map上的锥桶
    std::vector<fsd_common_msgs::Cone> UpdateCones_red;
    std::vector<fsd_common_msgs::Cone> UpdateCones_blue;
    std::vector<fsd_common_msgs::Cone> UpdateCones_yellow;
    std::vector<fsd_common_msgs::Cone> UpdateCones_unknow;

    map_.cone_red.clear();
    map_.cone_blue.clear();
    map_.cone_low_yellow.clear();
    map_.cone_high_yellow.clear();

    //拷贝当前扫描数据到各类型的ScanCones中
    for(i = 0;i<laserCloudMsg -> cone_red.size();i++){
        ScanCones_red.push_back(laserCloudMsg -> cone_red[i]);
    }

    for(i = 0;i<laserCloudMsg -> cone_blue.size();i++){
        ScanCones_blue.push_back(laserCloudMsg -> cone_blue[i]);
    }

    for(i = 0;i<laserCloudMsg -> cone_yellow.size();i++){
        ScanCones_yellow.push_back(laserCloudMsg -> cone_yellow[i]);
    }

    for(i = 0;i<laserCloudMsg -> cone_unknow.size();i++){
        ScanCones_unknow.push_back(laserCloudMsg -> cone_unknow[i]);
    }
    
    ROS_INFO("ScanCones Successful");

    //进行已知位姿的建图
    UpdateMap(now_stamp, ScanCones_red, UpdateCones_red, HistoryUpdateCone_red);
    UpdateMap(now_stamp, ScanCones_blue, UpdateCones_blue, HistoryUpdateCone_blue);
    UpdateMap(now_stamp, ScanCones_yellow, UpdateCones_yellow, HistoryUpdateCone_yellow);
    UpdateMap(now_stamp, ScanCones_unknow, UpdateCones_unknow, HistoryUpdateCone_unknow);

    //增加聚类簇
    generate_CenterCone(CenterCone_red, UpdateCones_red);
    generate_CenterCone(CenterCone_blue, UpdateCones_blue);
    generate_CenterCone(CenterCone_yellow, UpdateCones_yellow);
    generate_CenterCone(CenterCone_unknow, UpdateCones_unknow);
    
    //聚类时，是对Map上所有点进行聚类，Map上的UpdateCones根据CenterCone进行聚类，最终得到ClusterCone
    SecondCluster(ClusterCone_red, CenterCone_red, UpdateCones_red,HistoryUpdateCone_red);
    SecondCluster(ClusterCone_blue, CenterCone_blue, UpdateCones_blue, HistoryUpdateCone_blue);
    SecondCluster(ClusterCone_yellow, CenterCone_yellow, UpdateCones_yellow, HistoryUpdateCone_yellow);
    SecondCluster(ClusterCone_unknow, CenterCone_unknow, UpdateCones_unknow, HistoryUpdateCone_unknow);
    
    //将二次聚类点转到fsd_msgs::Map上
    Switch_to_Map(ClusterCone_red,ClusterCone_blue, ClusterCone_yellow, ClusterCone_unknow);

    //map可视化处理
    Visualization_Map();

    //发布消息
    Msg_to_Pub();
    ROS_INFO("laserCallback Successful");
}

/*得到对应时刻t的车辆位姿*/
void SlamMapping::getVehiclePose(Mapping::OrientedPoint& Vehicle_pose, const ros::Time& t){
    int index_to_time = 0;
    for(int i=0;i<TimeSeries.size();i++){
        if(t ==TimeSeries[i])
        index_to_time = i;
    }
    Vehicle_pose = VehiclePose[index_to_time];
    //如果得不到index_to_time的话，要返回false，但要避免index_to_time==0的情况
}

/*地图更新,*/
//UpdateMap，将ScanPoints扫描到的点，转为Map上的点，然后pushback到mMapPoints上
void SlamMapping::UpdateMap(const ros::Time& t, const  std::vector<fsd_common_msgs::Cone> &ScanCones, std::vector<fsd_common_msgs::Cone> &UpdateCones, std::vector<fsd_common_msgs::Cone> &HistoryCones)
{
    ROS_DEBUG("Update map");
    //做已知定位的负责建图，Vehicle_pose表示当前车辆位姿
    Mapping::OrientedPoint Vehicle_pose;
    Vehicle_pose.x = carstate_.car_state.x;
    Vehicle_pose.y = carstate_.car_state.y;
    Vehicle_pose.theta = carstate_.car_state.theta;
    //getVehiclePose(Vehicle_pose, t);

    //x轴纵向，y轴横向，z轴朝上
    double mx , my;
    fsd_common_msgs::Cone updateCone;
    if (ScanCones.size() == 0){
        ROS_INFO("UpdateCones default Successful");
        return;
    }
    else{
        updateCone.color = ScanCones[0].color;
    }
    
    for (int i=0;i<ScanCones.size();i++){
        mx = ScanCones[i].position.x;
        my = ScanCones[i].position.y;
        updateCone.position.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) - my * sin(Vehicle_pose.theta);
        updateCone.position.y = Vehicle_pose.y + mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
        UpdateCones.push_back(updateCone);
        HistoryCones.push_back(updateCone);
        if(updateCone.position.x > 200){
            ROS_INFO("UpdateCones OutOfDist");
        }
    }
    ROS_INFO("UpdateCones Successful");
}

void SlamMapping::generate_CenterCone(std::vector<fsd_common_msgs::Cone> &mCenterCone, std::vector<fsd_common_msgs::Cone> &UpdateCones){
    //激光第一帧的一次聚类点全算中心点
    //输入当前扫描到的laserCloudMsg.cone
    if(UpdateCones.size() == 0){
        ROS_INFO("generate_CenterCone default Successful");
        return;
    }

    if(laser_number_ == 1){
        for(int i =0;i<UpdateCones.size();i++){
            mCenterCone.push_back(UpdateCones[i]);
        }
    }

    else{
        double thershold_dist = 2;
        bool isCenter = true;
        for(int i = 0;i<UpdateCones.size();i++){
            for(int j = 0;j<mCenterCone.size();j++){
                double dist = Caculate_Dist(mCenterCone[j], UpdateCones[i]);
                if(dist < thershold_dist){
                    isCenter = false;
                }               
            }
            if(isCenter){
                mCenterCone.push_back(UpdateCones[i]);
            }
        }
    }
    ROS_INFO("generate_CenterCone Successful");
}

//计算两个锥桶间的欧式距离
double SlamMapping::Caculate_Dist(const fsd_common_msgs::Cone &Cone1, const fsd_common_msgs::Cone &Cone2){
    double distance;
    geometry_msgs::Point p1 = Cone1.position;
    geometry_msgs::Point p2 = Cone2.position;
    distance = hypot(p1.x-p2.x, p1.y-p2.y);
    return distance;
}

/*二次聚类，输入为CalibratedMapPoints和当前簇数*/
void SlamMapping::SecondCluster(std::vector<fsd_common_msgs::Cone> &mClusterCones, const std::vector<fsd_common_msgs::Cone> &CenterCones, std::vector<fsd_common_msgs::Cone> &UpdateCones, std::vector<fsd_common_msgs::Cone> &HistoryCones){
    //按照当前的传入数据点，判断一共有多少簇，初始化K个中心点
    //每一帧，SortOutCones都要清空一次，重新定义一遍
    //label从0到K，代表K个簇

    int label_cluster = 0;
    K = CenterCones.size();
    ROS_INFO("CenterCones has %d Class", K);
    mClusterCones.clear();
    
    if(UpdateCones.size()==0){
        ROS_INFO("SecondCluster UpdateCones default Successful");
        return;
    }
    else{
        if(K == 0){
            ROS_INFO("SecondCluster CenterCones default Successful");
            return;
        }
    }
    fsd_common_msgs::Cone SortCone;
    std::vector<std::vector<fsd_common_msgs::Cone>> SortOutCones(K);

    //如何将SortOutCones下的锥桶进行归类，
    SortCone.color = UpdateCones[0].color;

    //遍历UpdateCones上的点，进行归类，获得label_cluster，根据label_cluster，装进SortOutCones中
    for(int i =0; i < UpdateCones.size(); i++){
        label_cluster = clusterOfMap(CenterCones, UpdateCones[i]);
        SortCone.position.x = UpdateCones[i].position.x;
        SortCone.position.y = UpdateCones[i].position.y;
        SortCone.position.z = 0;
        //怎么把分好类的label的UpdateCones，push到SortOutCones里
        SortOutCones[label_cluster].push_back(SortCone);
    }

    for(int i=0;i<HistoryCones.size();i++){
        label_cluster = clusterOfMap(CenterCones, HistoryCones[i]);
        SortCone.position.x = HistoryCones[i].position.x;
        SortCone.position.y = HistoryCones[i].position.y;
        SortCone.position.z = 0;
        //怎么把分好类的label的HistoryCones，push到SortOutCones里
        SortOutCones[label_cluster].push_back(SortCone);
        }

    //根据SortOutCones中每一个簇，归纳出一个中心点
    getClusterPoints(SortOutCones, mClusterCones);

    //到这里，ClusterMapPoints肯定是聚类完了，而且应该是各点的均值，含有label值
    got_clustered_ = true;
    ROS_INFO("SecondCluster Successful");
}

//根据质心，决定当前点属于哪个簇
int SlamMapping::clusterOfMap(const std::vector<fsd_common_msgs::Cone> &CenterCones, const fsd_common_msgs::Cone &mUpdateCones) {
	float dist = Caculate_Dist(CenterCones[0], mUpdateCones);
	float tmp;
	int label = 0;
	for (int i = 1; i < CenterCones.size(); i++) {
		tmp = Caculate_Dist(CenterCones[i], mUpdateCones);
		if (tmp<dist) { dist = tmp; label = i; }
	}
	return label;
}

//取出各簇的点的平均值
void SlamMapping::getClusterPoints(std::vector<std::vector<fsd_common_msgs::Cone>> SortOutCones, std::vector<fsd_common_msgs::Cone> &mClusterCones){
    double x_ave;
    double y_ave;
    fsd_common_msgs::Cone AverageCone;

    if(SortOutCones.size()==0){
        ROS_INFO("getClusterPoints No.1 default Successful"); 
        return;
    }
    else{
        for(int i=0;i<SortOutCones.size();i++){
            if(SortOutCones[i].size() > 0){
                AverageCone.color = SortOutCones[i][0].color;
            }
        }
    }
    
    for(int i=0;i<K;i++ ){
        double x_sum=0;
        double y_sum=0;
        if(SortOutCones[i].size() >= 1){
            for(int j=0;j<SortOutCones[i].size();j++){
                x_sum += SortOutCones[i][j].position.x;
                y_sum += SortOutCones[i][j].position.y;
            }
            x_ave = x_sum / SortOutCones[i].size();
            y_ave = y_sum / SortOutCones[i].size();
            AverageCone.position.x = x_ave;
            AverageCone.position.y = y_ave;
            AverageCone.position.z = 0;

            if(AverageCone.position.x > 200){
            ROS_INFO("getClusterPoints OutOfDist");
            }
            mClusterCones.push_back(AverageCone);            
        }
    }
    ROS_INFO("getClusterPoints Successful");  
        
}
