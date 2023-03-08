#include <formation_layer/formation_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(formation_layer_namespace::FormationLayer, costmap_2d::Layer)

// using costmap_2d::LETHAL_OBSTACLE; 
using costmap_2d::FREE_SPACE;
namespace formation_layer_namespace
{
    FormationLayer::FormationLayer() : nh_("~") {}
    
    void FormationLayer::onInitialize()
    {
        ros::NodeHandle nh_("~/"+ name_);
        current_ = true;
        //Plugin initialization
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&FormationLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        //Subscribers to the robots footprints
        robot_pose_sub_0_= nh_.subscribe("/robot0/amcl_pose",10, &FormationLayer::robotposeCallback_0, this);
        robot_fp_subs_0_ = nh_.subscribe("/robot0/move_base_flex/local_costmap/footprint", 10, &FormationLayer::robotFPCallback_0, this);
        robot_fp_subs_1_ = nh_.subscribe("/robot1/move_base_flex/local_costmap/footprint", 10, &FormationLayer::robotFPCallback_1, this);
        robot_fp_subs_2_ = nh_.subscribe("/robot2/move_base_flex/local_costmap/footprint", 10, &FormationLayer::robotFPCallback_2, this);
    }

    void FormationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }   

    void FormationLayer::robotposeCallback_0(const geometry_msgs::PoseWithCovarianceStamped &msg){
        ROS_INFO("PoseCallback 0 started");
        mark_x_ = msg.pose.pose.position.x;
        mark_y_ = msg.pose.pose.position.y;
    }
    
    void FormationLayer::robotFPCallback_0(const geometry_msgs::Polygon &msg)
    {
        ROS_INFO("Callback 0 started");
        for(const auto& point : msg.points){
            geometry_msgs::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            fp_0.push_back(p);
            ROS_INFO("Callback 0 done"); 
        }
    }

    void FormationLayer::robotFPCallback_1(const geometry_msgs::Polygon &msg){
        ROS_INFO("Callback 1 started");
        for(const auto& point : msg.points){
            geometry_msgs::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            fp_1.push_back(p);
            ROS_INFO("Callback 1 done"); 
        }
    }

    void FormationLayer::robotFPCallback_2(const geometry_msgs::Polygon &msg){
        ROS_INFO("Callback 2 started");
        for(const auto& point : msg.points){
            geometry_msgs::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            fp_1.push_back(p);
            ROS_INFO("Callback 2 done"); 
        }
    }

    // void AreaLayer::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
    // {
    //     int dx = abs(x1 - x0);
    //     int dy = abs(y1 - y0);
    //     PointInt pt;
    //     pt.x = x0;
    //     pt.y = y0;
    //     int n = 1 + dx + dy;
    //     int x_inc = (x1 > x0) ? 1 : -1;
    //     int y_inc = (y1 > y0) ? 1 : -1;
    //     int error = dx - dy;
    //     dx *= 2;
    //     dy *= 2;
    //     for (; n > 0; --n) {
    //         cells.push_back(pt);
    //         if (error > 0.0) {
    //             pt.x += x_inc;
    //             error -= dy;
    //         } else {
    //             pt.y += y_inc;
    //             error += dx;
    //         }
    //     }
    // }

    // void AreaLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
    // {
    //     for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    //         linetrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
    //     }
    //     if (!polygon.empty()) {
    //         unsigned int last_index = polygon.size() - 1;
    //         // we also need to close the polygon by going from the last point to the first
    //         linetrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
    //     }
    // }

    void FormationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if(!enabled_)
            return;

        // mark_x_ = robot_x;
        // mark_y_ = robot_y;

        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
        ROS_INFO("UpdateBounds done");
    }

    void FormationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if(!enabled_)
            return;
        unsigned int mx;
        unsigned int my;
        if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
            //  master_grid.setCost(mx, my, LETHAL_OBSTACLE);
             master_grid.setCost(mx, my, FREE_SPACE);
             ROS_INFO("SetCost done");
        }
    }//end_namespace
}