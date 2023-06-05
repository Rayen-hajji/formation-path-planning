#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <iostream>
#include <functional>
#include <map>
#include <cmath>
#include <dynamic_reconfigure/Reconfigure.h>
#include <fpp_msgs/DynReconfigure.h>
#include <fpp_msgs/FormationFootprintInfo.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/client.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdint>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <algorithm>

nav_msgs::OccupancyGrid costmap0,costmap1,costmap2, costmap_merged;
bool costmap0_received = false;
bool costmap1_received = false;
bool costmap2_received = false;

void costmapCallback0(const nav_msgs::OccupancyGridConstPtr& msg){

    costmap0.header = msg->header;
    costmap0.info = msg->info;
    costmap0.data.resize(msg->data.size());
    ROS_INFO("costmap0Callback called!");
    for(int i=0; i<msg->data.size(); ++i){
        // costmap0.data.push_back(msg.data[i]);
        costmap0.data[i] = msg->data[i];
    }
    costmap0_received = true;
    ROS_INFO("costmap0_received : %d", costmap0_received);
}

void costmapCallback1(const nav_msgs::OccupancyGridConstPtr& msg){
    costmap1.header = msg->header;
    costmap1.info = msg->info;
    costmap1.data.resize(msg->data.size());
    for(int i=0; i<msg->data.size(); ++i){
        // costmap1.data.push_back(msg.data[i]);
        costmap1.data[i] = msg->data[i];
    }
    costmap1_received = true;
    ROS_INFO("costmap1_received : %d", costmap0_received);
}

void costmapCallback2(const nav_msgs::OccupancyGridConstPtr& msg){
    costmap2.header = msg->header;
    costmap2.info = msg->info;
    costmap2.data.resize(msg->data.size());
    for(int i=0; i<msg->data.size(); ++i){
        // costmap2.data.push_back(msg.data[i]);
        costmap2.data[i] = msg->data[i];
    }
    costmap2_received = true;
    ROS_INFO("costmap2_received : %d", costmap0_received);
}

int findMax(int value_1, int value_2,int value_3 ){
    int result = value_1;
    if(value_2 > result){
        result = value_2;
    }
    if(value_3 > result){
        result = value_3;
    }
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merger_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    

    sleep(10);
    ros::Subscriber costmapsSubscriber0 = nh.subscribe("/robot0/move_base_flex/global_costmap/costmap",10,&costmapCallback0);
    ros::Subscriber costmapsSubscriber1 = nh.subscribe("/robot1/move_base_flex/global_costmap/costmap",10,&costmapCallback1); 
    ros::Subscriber costmapsSubscriber2 = nh.subscribe("/robot2/move_base_flex/global_costmap/costmap",10,&costmapCallback2);
    ros::Publisher mergedcostmapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("merged_costmap",10,true);

    ROS_INFO("Node started!");
    while (ros::ok())
    {
        if((costmap0_received)&&(costmap2_received)){
            if(costmap0.data.size()!=costmap2.data.size()){
                ROS_ERROR("costmap 0 and 2 have different costmap sizes");
                return 0;
            }
            costmap_merged.header = costmap0.header;
            costmap_merged.info = costmap0.info;
            costmap_merged.data.resize(costmap0.data.size());
            ROS_INFO("merging starts!");
            for(int i =0; i < costmap0.data.size(); ++i){
                // costmap_merged.data.push_back(findMax(costmap0.data[i],costmap1.data[i],costmap2.data[i]));
                int max = findMax(costmap0.data[i],0,costmap2.data[i]);
                // ROS_INFO("max is : %d", max);
                // costmap_merged.data[i] = max;
                costmap_merged.data[i] = costmap0.data[i];               
            }
            ROS_INFO("first cost of merged costmap is : %d", costmap_merged.data[0]);
            mergedcostmapPublisher.publish(costmap_merged);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

