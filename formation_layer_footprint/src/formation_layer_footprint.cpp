#include <formation_layer_footprint/formation_layer_footprint.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(formation_layer_footprint_namespace::FormationLayerFootprint, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;

namespace formation_layer_footprint_namespace
{
    FormationLayerFootprint::FormationLayerFootprint() {} 
    
    void FormationLayerFootprint::onInitialize()
    {

        this->master_grids = vector<costmap_2d::Costmap2D>(); //for testing

        ROS_INFO("Global costmap using formation_layer_footprint plugin");
        this->nh_ = ros::NodeHandle("~/"+ name_);

        FormationLayerFootprint::matchSize();
        current_= true;

        //initialize the service flag
        this->service_flag = true;

        //initialize the previous_footprints vector
        this->previous_footprints = vector<polygon>();

        //subscribe to the formation footprint topic
        laserSub = this->nh_.subscribe("/robot2/scan",10,&FormationLayerFootprint::laserCallback, this); //for testing
        //publishers
        footprintsPub = this->nh_.advertise<geometry_msgs::PolygonStamped>("all_footprints",10,true);
        updateBoundsPub = this->nh_.advertise<geometry_msgs::PolygonStamped>("update_bounds",10,true);
        mecPub = this->nh_.advertise<visualization_msgs::Marker>("minimum_enclosing_circle",10,true);
        mecCenterPub = this->nh_.advertise<geometry_msgs::PointStamped>("minimum_enclosing_circle_center",10,true);
        boundingBoxPub = this->nh_.advertise<geometry_msgs::PolygonStamped>("bounding_box",10,true);
        InflationRadiusPub = this->nh_.advertise<std_msgs::Float64>("inflation_radius",10,true);
        markerPub = this->nh_.advertise<visualization_msgs::MarkerArray>("footprint_points_markers", 10, true);  //for testing
        leftPointsPub = this->nh_.advertise<visualization_msgs::MarkerArray>("left_points_markers", 10, true);  //for testing
        obstaclesPub = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_points_markers", 10, true); //for testing
        mec2Pub = this->nh_.advertise<visualization_msgs::Marker>("minimum_enclosing_circle_2",10,true); // for testing
        mcCenter2Pub = this->nh_.advertise<geometry_msgs::PointStamped>("minimum_enclosing_circle_center_2",10,true); //for Testing

        if(this->service_flag){
            this->mecService = this->nh_.advertiseService("mec_Info", &FormationLayerFootprint::mecInfoServiceCallback, this);
            this->service_flag = false;
        }


        //get the number of robots from the launch file
        std::string robots_number_key;
        if(this->nh_.searchParam("robots_number", robots_number_key)){
			this->nh_.getParam(robots_number_key, robots_number);
            ROS_INFO("Number of Robots is:%d", robots_number);
            

            for (int i = 0; i < robots_number; i++){

                //position topic
                std::string topic_name = "/robot" + std::to_string(i) + "/amcl_pose";
                
                // ROS_INFO("topic %d created : %s",i,topic_name.c_str());

                //robot ID
                string robot_id = "robot_" + to_string(i);
                
                //create Callback functions 
                callbacks.push_back ([this, i, robot_id](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
                    // this->robot_poses.push_back(*msg.get());
                    this->robot_positions[robot_id] = *msg;
                }); 

                //create and save subscribers 
                Subscribers.push_back(this->nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 10, callbacks[i]));

                //only for testing
                // ROS_INFO("Topic %d is %s", i, Subscribers[i].getTopic().c_str());
                if(i == robots_number-1){}
                    // ROS_INFO("all Subscribers are created");
                else ROS_INFO("Was in Loop");
            }
        }
		else
			ROS_ERROR("No Robots number parameter was found in the launch file.");

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

    void FormationLayerFootprint::laserCallback(const sensor_msgs::LaserScan &msg){
        this->laser_data.angle_max = msg.angle_max;
        this->laser_data.angle_min = msg.angle_min;
        this->laser_data.angle_increment = msg.angle_increment;
        this->laser_data.ranges = vector<double>();
        this->laser_frame = msg.header.frame_id; 
        for(const auto& distance : msg.ranges){
            this->laser_data.ranges.push_back(distance); 
        }
        // ROS_INFO("size of ranges is :%ld",laser_data.ranges.size());
     }     

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
    
    polygon FormationLayerFootprint::boundingbox(polygon &footprint){
        polygon boundingbox;
        double min_x = footprint[0].x;
        double min_y = footprint[0].y;
        double max_x = footprint[0].x;
        double max_y = footprint[0].y;
        for(int i=0; i < footprint.size(); i++){
            if(footprint[i].x > max_x)
                max_x = footprint[i].x;
            if(footprint[i].x < min_x)
                min_x = footprint[i].x;
            if(footprint[i].y > max_y)
                max_y = footprint[i].y;
            if(footprint[i].y < min_y)
                min_y = footprint[i].y;
        }
        geometry_msgs::Point p1,p2,p3,p4;
        p1.x = max_x;
        p1.y = max_y;
        boundingbox.push_back(p1);
        p2.x = max_x;
        p2.y = min_y;
        boundingbox.push_back(p2);
        p3.x = min_x;
        p3.y = min_y;
        boundingbox.push_back(p3);
        p4.x = min_x;
        p4.y = max_y;
        boundingbox.push_back(p4);
        return boundingbox;
    }

    void FormationLayerFootprint::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                        double* max_x, double* max_y)
    {
        if(!enabled_)
            return;
        
        // if(robot_positions.size() == robots_number){
        if(!robot_positions.empty()){ 
            this->footprint_points = vector<geometry_msgs::Point>();
            this->formation_fp_points = vector<Point>();
            ROS_INFO("UC:UpdateCosts started");
            
            ROS_INFO("robot_positions size is %ld", robot_positions.size());
            for(int i = 0 ; i < robot_positions.size(); i++){
                
                string robot_id = "robot_" + to_string(i);

                getUnitFootprint(robot_positions[robot_id], this->footprints[robot_id]);
                
                //save all footprints here
                this->previous_footprints.push_back(footprints[robot_id]);

                for(const auto& point : footprints[robot_id]){
                    this->footprint_points.push_back(point);
                }

                //for Testing:publish footprints
                geometry_msgs::PolygonStamped footprints_msg;
                footprints_msg.header.frame_id = "map";
                footprints_msg.header.stamp = ros::Time::now();
                for(const auto& point : footprints[robot_id]){
                    geometry_msgs::Point32 p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0;
                    footprints_msg.polygon.points.push_back(p);
                }
                this->footprintsPub.publish(footprints_msg);
            }

            for(int i=0; i < this->footprint_points.size(); i++){
                // ROS_INFO("UC: footprint_points[%d] = (%f,%f)",i,this->footprint_points[i].x,this->footprint_points[i].y);
            }
            //for testing: publish the footprint points as Markers
            visualization_msgs::MarkerArray marker_array;
            for (unsigned int i = 0; i < this->footprint_points.size(); ++i) {
                // Create a marker for each point
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";  // Set the frame ID of the markers
                marker.header.stamp = ros::Time::now();
                marker.ns = "point_markers";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position = this->footprint_points[i];  // Set the position of the marker based on the point coordinates
                marker.scale.x = 0.1;  // Set the size of the marker
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 1.0;  // Set the color of the marker
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;  // Set the alpha (transparency) of the marker

                // Add the marker to the marker array
                marker_array.markers.push_back(marker);
            }
            this->markerPub.publish(marker_array);
            
            //create the formation footprint from the footprints points
            this->formation_footprint = findConvexHull(this->footprint_points, this->footprint_points.size());

            //convert the formation footprint to points
            for(const auto& point : this->formation_footprint){
                Point p;
                p.X = point.x;
                p.Y = point.y;
                this->formation_fp_points.push_back(p);
            }

            //get the minimum enclosing circle
            this->mec = welzl(formation_fp_points);
            geometry_msgs::PointStamped mecC_msg;
            mecC_msg.header.frame_id = "map";
            mecC_msg.point.x = mec.C.X;
            mecC_msg.point.y = mec.C.Y;
            mecC_msg.point.z = 0.0;
            this->mecCenterPub.publish(mecC_msg);
            reconfigureInflationRadius(mec.R * 1.5);

            //publish the formaiton minimum ecnlosing circle
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = mec.C.X;
            marker.pose.position.y = mec.C.Y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 2.0 * mec.R;
            marker.scale.y = 2.0 * mec.R;
            marker.scale.z = 0.1;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            mecPub.publish(marker);

            //publish formation footprint
            formationFPPub = this->nh_.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10, true);
            geometry_msgs::PolygonStamped formation_footprint_msg;
            formation_footprint_msg.header.frame_id = "map";
            formation_footprint_msg.header.stamp = ros::Time::now();
            for(const auto& point : this->formation_footprint){
                geometry_msgs::Point32 p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0;
                formation_footprint_msg.polygon.points.push_back(p);
            }
            formationFPPub.publish(formation_footprint_msg);

            //get the BoundingBox of the formation footprint
            this->bounding_box = vector<geometry_msgs::Point>();
            this->bounding_box = boundingbox(formation_footprint);
            *min_x = bounding_box[2].x; 
            *min_y = bounding_box[2].y;
            *max_x = bounding_box[0].x;
            *max_y = bounding_box[0].y;

            //publish the bouding box to visualize
            geometry_msgs::PolygonStamped boundingBoxMsg;
            boundingBoxMsg.header.frame_id = "map";
            boundingBoxMsg.header.stamp = ros::Time::now();
            for(const auto& point : this->bounding_box){
                geometry_msgs::Point32 p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0;
                boundingBoxMsg.polygon.points.push_back(p);
            }
            boundingBoxPub.publish(boundingBoxMsg);
        }
    }
        
    void FormationLayerFootprint::setPolygonCost(costmap_2d::Costmap2D &master_grid, const polygon &polygon,
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
            //check if point is outside bounds
            if (mx < min_i || mx >= max_i)
                continue;
            if (my < min_j || my >= max_j)
                continue;
            master_grid.setCost(mx, my, cost);
        }
    }     

    void FormationLayerFootprint::getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, polygon &RobotFootprint){
        // ROS_INFO("getUnitFootprint started");
        geometry_msgs::PolygonStamped RobotFootprintStamped;
        //get yaw 
        geometry_msgs::Quaternion orientation = position.pose.pose.orientation;
        tf::Quaternion tf_quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        double yaw = tf::getYaw(tf_quaternion);
        //build the oriented footprint 
        costmap_2d::transformFootprint(position.pose.pose.position.x, position.pose.pose.position.y,
                        yaw, getFootprint(), RobotFootprintStamped );
        // ROS_INFO("position is (%f,%f,%f)",position.pose.pose.position.x,position.pose.pose.position.y,position.pose.pose.orientation.z);
        RobotFootprint.clear();
        for(const auto& point : RobotFootprintStamped.polygon.points){
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            RobotFootprint.push_back(p);  
        }
    }    

    void FormationLayerFootprint::reconfigureInflationRadius(double radius){
        fpp_msgs::DynReconfigure::Request req;
        req.new_inflation_radius = radius;
        req.robot_namespace = "/robot0";

        dynamic_reconfigure::Reconfigure dyn_reconfigure_inflation_msg;
        dynamic_reconfigure::DoubleParameter inflation_reconfig;
        inflation_reconfig.name = "inflation_radius";
        inflation_reconfig.value = req.new_inflation_radius;
        ROS_INFO_STREAM("New inflation radius reconfigured from formation Layer: " << req.new_inflation_radius);
        dyn_reconfigure_inflation_msg.request.config.doubles.push_back(inflation_reconfig);
        
        ROS_INFO_STREAM(req.robot_namespace << "/move_base_flex/global_costmap/inflation/set_parameters");
        ros::service::call(req.robot_namespace + "/move_base_flex/global_costmap/inflation/set_parameters", 
                            dyn_reconfigure_inflation_msg.request, 
                            dyn_reconfigure_inflation_msg.response);
    }

    bool FormationLayerFootprint::mecInfoServiceCallback(fpp_msgs::FormationFootprintInfo::Request &req, fpp_msgs::FormationFootprintInfo::Response &res){
        res.formation_centre.x = this->mec.C.X;
        res.formation_centre.y = this->mec.C.Y;
        res.formation_centre.theta = 0.0;
        res.minimal_encloring_circle_radius = this->mec.R;
        return true;
    }

    void FormationLayerFootprint::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if(!enabled_)
            return;
        
        ROS_INFO("updateCost started");

        //for testing 
        // this->obstacle_map = vector<geometry_msgs::Point>();
        // this->flag = false;
        // int cols = master_grid.getSizeInCellsY();
        // int rows = master_grid.getSizeInCellsX();
        // this->origin_x =  master_grid.getOriginX();
        // this->origin_y =  master_grid.getOriginY();
        // this->resolution = master_grid.getResolution();

        if(this->footprints.size() == robots_number){
            for(int i = 0 ; i < robots_number; i++){
                string robot_id = "robot_" + to_string(i);
                setPolygonCost(master_grid, this->footprints[robot_id], FREE_SPACE, min_i, min_j, max_i, max_j, true);
                // save all footprints here
                // this->previous_footprints.push_back(footprints[robot_id]);
                // for(int i=0 ; i < previous_footprints.size(); i++){
                //     setPolygonCost(master_grid, this->previous_footprints[i], FREE_SPACE, min_i, min_j, max_i, max_j, true);
                // }
                // if(this->previous_footprints.size()>300)
                //     this->previous_footprints.erase(this->previous_footprints.begin(),this->previous_footprints.end());
            }

        //     this->cells = vector<PointInt> ();
        //     master_grids.push_back(master_grid);
        //     ROS_INFO("master_grids size is %ld",master_grids.size());  
        //     if(master_grids.size()>1){
        //         //get the laser points
        //         this->obstacles = vector<geometry_msgs::Point>();
        //         for(int i= 0; i < this->laser_data.ranges.size(); ++i){
        //             double angle = i * this->laser_data.angle_increment + this->laser_data.angle_min;
        //             geometry_msgs::Point point;
        //             point.x = this->laser_data.ranges[i] * cos(angle);
        //             point.y = this->laser_data.ranges[i] * sin(angle);
        //             point.z = 0;
        //             obstacles.push_back(point);
        //         }
        //         if(!this->obstacles.empty()){
        //             //transformation 
        //             ROS_INFO("transformation started");
        //             tf2_ros::Buffer tfBuffer;
        //             tf2_ros::TransformListener tfListener(tfBuffer);
        //             geometry_msgs::TransformStamped transformStamped;
        //             // this->obstacle_map = vector<geometry_msgs::Point>();
        //             try{
        //                 transformStamped = tfBuffer.lookupTransform("map",this->laser_frame,ros::Time(0), ros::Duration(1.0));
        //                 for(int i = 0; i < this->obstacles.size(); ++i){
        //                     geometry_msgs::Point map_point;
        //                     tf2::doTransform(this->obstacles[i],map_point,transformStamped);
        //                     this->obstacle_map.push_back(map_point);
        //                 }
        //             }
        //             catch (tf2::TransformException& ex) {
        //                 ROS_ERROR("Failed to perform transformation: %s", ex.what());
        //             }
        //         }
        //         //get the changing points from grid to grid
        //         for(int i = 0; i < cols; ++i){
        //             for(int j = 0; j < rows; ++j){
        //                 if(master_grids[0].getCost(i,j) != master_grids[1].getCost(i,j)){
        //                     PointInt different_cell;
        //                     different_cell.x = i;
        //                     different_cell.y = j;
        //                     this->cells.push_back(different_cell);
        //                 }
        //             }
        //         }
        //         if(!cells.empty()){
        //             for(int i = 0; i <this->cells.size(); ++i){
        //                 geometry_msgs::Point leftPoint;
        //                 leftPoint.x = this->origin_x + (this->cells[i].x + 0.5) * this->resolution;
        //                 leftPoint.y = this->origin_y + (this->cells[i].y + 0.5) * this->resolution;
        //                 if((leftPoint.x<this->bounding_box[0].x)&&(leftPoint.x>this->bounding_box[2].x)&&(leftPoint.y<this->bounding_box[0].y)&&(leftPoint.y>this->bounding_box[2].y)){
        //                     ROS_INFO(" left cell in (%f,%f)",leftPoint.x,leftPoint.y);
        //                     this->flag = true;
        //                 }
        //                 if (this->flag)
        //                 break;
        //             }
        //         }

        //         //publish the laser points when change occured 
        //         if(this->flag){   
        //             visualization_msgs::MarkerArray obstacles_marker_array;
        //             for (unsigned int i = 0; i < this->obstacle_map.size(); ++i){
        //                 if((obstacle_map[i].x<this->bounding_box[0].x+5)&&(obstacle_map[i].x>this->bounding_box[2].x-5)&&(obstacle_map[i].y<this->bounding_box[0].y+5)&&(obstacle_map[i].y>this->bounding_box[2].y-5)){
        //                     // if(true){    
        //                         // Create a marker for each point
        //                         visualization_msgs::Marker marker;
        //                         marker.header.frame_id = "map";  // Set the frame ID of the markers
        //                         marker.header.stamp = ros::Time::now();
        //                         marker.ns = "obstacles_markers";
        //                         marker.id = i;
        //                         marker.type = visualization_msgs::Marker::SPHERE;
        //                         marker.action = visualization_msgs::Marker::ADD;
        //                         marker.pose.position = this->obstacle_map[i];  // Set the position of the marker based on the point coordinates
        //                         marker.scale.x = 0.1;  // Set the size of the marker
        //                         marker.scale.y = 0.1;
        //                         marker.scale.z = 0.1;
        //                         marker.color.r = 1.0;  // Set the color of the marker
        //                         marker.color.g = 0.0;
        //                         marker.color.b = 0.0;
        //                         marker.color.a = 1.0;  // Set the alpha (transparency) of the marker
        //                         // Add the marker to the marker array
        //                         obstacles_marker_array.markers.push_back(marker);
        //                 }
        //             }
        //             this->obstaclesPub.publish(obstacles_marker_array);
        //             obstacles_marker_array.markers.clear();
        //         }
        //         master_grids.erase(master_grids.begin());
        //     }
        }
        ROS_INFO("updateCost done");
    }
}//end_namespace 
            