#include <formation_layer_footprint/formation_layer_footprint.h>
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(formation_layer_footprint_namespace::FormationLayerFootprint, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
namespace formation_layer_footprint_namespace
{
    FormationLayerFootprint::FormationLayerFootprint() {} 
    
    void FormationLayerFootprint::onInitialize()
    {
        ROS_INFO("FormationLayerFootprint::onInitialize");
        this->nh_ = ros::NodeHandle("~/"+ name_);
        // rolling_window_ = layered_costmap_->isRolling();

        // Initializing vectors
        this->robot_poses = vector<geometry_msgs::PoseWithCovarianceStamped>();
        // this->robots_footprints = vector<Polygon>();

        FormationLayerFootprint::matchSize();
        current_= true;

        //get the number of robots from the launch file
        std::string robots_number_key;
        if(this->nh_.searchParam("robots_number", robots_number_key)){
			this->nh_.getParam(robots_number_key, robots_number);
            ROS_INFO("Number of Robots is:%d", robots_number);

            for (int i = 0; i < robots_number; i++){

                //position topic to subscribe to
                std::string topic_name = "/robot" + std::to_string(i) + "/amcl_pose";
                ROS_INFO("topic %d created : %s",i,topic_name.c_str());
                
                //create Callback function 
                callbacks.push_back ([this, i](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
                    ROS_INFO("%d Callback",i);
                    this->robot_poses.push_back(*msg.get());
                    // robot_poses[i].pose.pose.position.x = msg->pose.pose.position.x;
                    // robot_poses[i].pose.pose.position.y = msg->pose.pose.position.y;
                    // robot_poses[i].pose.pose.orientation.w = msg->pose.pose.orientation.w;
                    ROS_INFO("Callback function %d is created",i);
                    ROS_INFO("Robot position is : (%f,%f)",robot_poses[i].pose.pose.position.x,robot_poses[i].pose.pose.position.y);
                });

                //Subscriber
                Subscribers.push_back(this->nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 10, callbacks[i]));

                //Only for testing
                ROS_INFO("Topic %d is %s", i, Subscribers[i].getTopic().c_str());
                if(i == robots_number-1)
                    ROS_INFO("all Subscribers are created");
                else ROS_INFO("Was in Loop");
            }
        }
		else
			ROS_ERROR("No Robots number parameter was found.");

        //create the subscribers to the all the units positions
        // for (int i = 0; i < RobotsNumber; i++){
        //     //position topic to subscribe to
        //     std::string topic_name = "/robot" + std::to_string(i) + "/amcl_pose";
        //     ROS_INFO("topic %d created : %s",i,topic_name.c_str());
        //     //Subscriber
        //     Subscribers.push_back(this->nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 10, 
        //         [this,i](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        //             ROS_INFO("%d st subscriber",i);
        //             Robotposes[i].pose.pose.position.x = msg->pose.pose.orientation.x;
        //             Robotposes[i].pose.pose.position.y = msg->pose.pose.position.y;
        //             Robotposes[i].pose.pose.orientation.w = msg->pose.pose.orientation.w;
        //             ROS_INFO_STREAM("position of the Robot"<<i<<"is"<<Robotposes[i].pose.pose.position.x);
        //         }
        //     ));
        //     ROS_INFO("Topic %d is %s", i, Subscribers[i].getTopic().c_str());
        //     ROS_INFO("number of subscribers %d is %d", i, Subscribers[i].getNumPublishers());
        //     if(i == RobotsNumber-1)
        //         ROS_INFO("all Subscribers are created");
        //     else ROS_INFO("Was in Loop");
        // }
        // PoseSubscriber_0 = this->nh_.subscribe("/robot0/amcl_pose", 10, &FormationLayerFootprint::poseCallback_0, this );
        //Plugin initialization

        dsrv_ = NULL;
        setupDynamicReconfigure(nh_);
        ROS_INFO("FormationLayerFootprint::onInitialize END");
    }

    void FormationLayerFootprint::setupDynamicReconfigure(ros::NodeHandle& nh_){
        dsrv_ = new dynamic_reconfigure::Server<formation_layer_footprint::FormationLayerFootprintConfig>(nh_);
        dynamic_reconfigure::Server<formation_layer_footprint::FormationLayerFootprintConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);
    }

    FormationLayerFootprint::~FormationLayerFootprint(){
    if (dsrv_)
        delete dsrv_;
    } 

    void FormationLayerFootprint::reconfigureCB(formation_layer_footprint::FormationLayerFootprintConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }      

    // void FormationLayerFootprint::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    // {
    //     ROS_INFO("poseCallback_0 is called");
    //     ROS_INFO_STREAM(msg->header.frame_id);
    //     this->robot_poses.push_back(*msg.get());
    //     // this->robot_poses[0] = *msg.get();
    //     // robot_poses[0].pose.pose.position.x = msg->pose.pose.orientation.x;
    //     // robot_poses[0].pose.pose.position.y = msg->pose.pose.position.y;
    //     // robot_poses[0].pose.pose.orientation.w = msg->pose.pose.orientation.w;
    //     ROS_INFO("poseCallback_0 is finished");
    // }

    void FormationLayerFootprint::getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, Polygon RobotFootprint){
        ROS_INFO("getUnitFootprint started");
        geometry_msgs::PolygonStamped RobotFootprintStamped;
        costmap_2d::transformFootprint(position.pose.pose.position.x, position.pose.pose.position.y,
                        position.pose.pose.orientation.w, getFootprint() , RobotFootprintStamped );
        RobotFootprint.clear();
        for(const auto& point : RobotFootprintStamped.polygon.points){
            geometry_msgs::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            RobotFootprint.push_back(p);     
        }
        ROS_INFO("Footprint created, first point =(%f,%f)",RobotFootprint[0].x,RobotFootprint[0].y);
    }

    // void FormationLayerFootprint::getUnitFootprint_0(const geometry_msgs::PoseWithCovarianceStamped &position){
    //     costmap_2d::transformFootprint(position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.orientation.w, getFootprint() , RobotFootprintStamped_0 );
    //     RobotFootprint_0.clear();
    //     for(const auto& point : RobotFootprintStamped_0.polygon.points){
    //         geometry_msgs::Point32 p;
    //         p.x = point.x;
    //         p.y = point.y;
    //         p.z = 0;
    //         RobotFootprint_0.push_back(p);     
    //     }
    // ROS_INFO("getUnitFootprint Done first FP point =(%f,%f)", RobotFootprint_0[0].x, RobotFootprint_0[0].y); 
    // }

    void FormationLayerFootprint::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
    {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
    }

    void FormationLayerFootprint::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
    {
        for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
            linetrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
        }
        if (!polygon.empty()) {
            unsigned int last_index = polygon.size() - 1;
            // we also need to close the polygon by going from the last point to the first
            linetrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
        }
    }

    void FormationLayerFootprint::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
    {
        // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

        //we need a minimum polygon of a traingle
        if (polygon.size() < 3)
            return;

        //first get the cells that make up the outline of the polygon
        polygonOutlineCells(polygon, polygon_cells);

        if (!fill)
            return;

        //quick bubble sort to sort points by x
        PointInt swap;
        unsigned int i = 0;
        while (i < polygon_cells.size() - 1) {
            if (polygon_cells[i].x > polygon_cells[i + 1].x) {
                swap = polygon_cells[i];
                polygon_cells[i] = polygon_cells[i + 1];
                polygon_cells[i + 1] = swap;

                if (i > 0)
                    --i;
            } else
                ++i;
        }

        i = 0;
        PointInt min_pt;
        PointInt max_pt;
        int min_x = polygon_cells[0].x;
        int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

        //walk through each column and mark cells inside the polygon
        for (int x = min_x; x <= max_x; ++x) {
            if (i >= (int)polygon_cells.size() - 1)
                break;

            if (polygon_cells[i].y < polygon_cells[i + 1].y) {
                min_pt = polygon_cells[i];
                max_pt = polygon_cells[i + 1];
            } else {
                min_pt = polygon_cells[i + 1];
                max_pt = polygon_cells[i];
            }

            i += 2;
            while (i < polygon_cells.size() && polygon_cells[i].x == x) {
                if (polygon_cells[i].y < min_pt.y)
                    min_pt = polygon_cells[i];
                else if (polygon_cells[i].y > max_pt.y)
                    max_pt = polygon_cells[i];
                ++i;
            }

            PointInt pt;
            //loop though cells in the column
            for (int y = min_pt.y; y < max_pt.y; ++y) {
                pt.x = x;
                pt.y = y;
                polygon_cells.push_back(pt);
            }
        }
    }
    
    void FormationLayerFootprint::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if(!enabled_)
            return;
        ROS_INFO("UpdateBounds started");
        mark_x_ = robot_x;
        mark_y_ = robot_y;

        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
        ROS_INFO("UpdateBounds done");
    }

    void FormationLayerFootprint::setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon,
                        unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
    {   
        std::vector<PointInt> map_polygon;
        for (unsigned int i = 0; i < polygon.size(); ++i) {
            PointInt loc;
            master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
            map_polygon.push_back(loc);
        }

        std::vector<PointInt> polygon_cells;

        // get the cells that fill the polygon
        rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

        // set the cost of those cells
        for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
            int mx = polygon_cells[i].x;
            int my = polygon_cells[i].y;
            // check if point is outside bounds
            if (mx < min_i || mx >= max_i)
                continue;
            if (my < min_j || my >= max_j)
                continue;
            master_grid.setCost(mx, my, cost);
        }
    } 

    void FormationLayerFootprint::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        ROS_INFO("UpdateCosts started");
        if(!enabled_)
            return;
        this->robots_footprints = vector<Polygon>();
        for(int i = 0; i < robots_number; i++){
            Polygon footprint;
            ROS_INFO("getUnitFootprint from updateCosts started");
            ROS_INFO("position of robot %d is %f, %f", i, robot_poses[i].pose.pose.position.x, robot_poses[i].pose.pose.position.y);
            getUnitFootprint(this->robot_poses[i], footprint); // this->robots_footprints[i]
            ROS_INFO("setPolygonCost from updateCosts started");
            setPolygonCost(master_grid, footprint, FREE_SPACE, min_i, min_j, max_i, max_j, true); //RobotFootprints[i]
            footprint.clear();
        }
        ROS_INFO("UpdateCosts done");
    }
}//end_namespace                                                                   