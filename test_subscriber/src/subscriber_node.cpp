#include "ros/ros.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <vector>


using namespace std;

void robotFPCallback_0(const geometry_msgs::PolygonStamped &msg)
{
    std::vector<geometry_msgs::Point32> fp;
    for(const auto& point_1 : msg.polygon.points){
        geometry_msgs::Point32 p;
        p.x = point_1.x;
        p.y = point_1.y;
        p.z = 0;
        fp.push_back(p); 
    }
    for(const auto& point_2 : fp){
        ROS_INFO("pos x =%f , pos y=%f , pos=%f",point_2.x,point_2.y,point_2.z);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "FPSubscriber");

    ros::NodeHandle n;

    ros::Subscriber robot_fp_subs_0_ = n.subscribe("/robot0/move_base_flex/global_costmap/footprint", 10, robotFPCallback_0);

    ros::spin();

    return 0;

}