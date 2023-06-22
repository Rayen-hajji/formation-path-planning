#include <formation_layer/formation_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(formation_layer_namespace::FormationLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;

namespace formation_layer_namespace
{
    FormationLayer::FormationLayer() {} 
    
    void FormationLayer::onInitialize()
    {

        ROS_INFO("Global costmap using formation_layer plugin");
        this->nh_ = ros::NodeHandle("~/"+ name_);

        FormationLayer::matchSize();
        current_= true;

        //initialize the service flag
        this->service_flag_ = true;
          
        formationFPPub_ = this->nh_.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10, true); 
        mecPub_ = this->nh_.advertise<visualization_msgs::Marker>("minimum_enclosing_circle",10,true);
        boundingBoxPub_ = this->nh_.advertise<geometry_msgs::PolygonStamped>("bounding_box",10,true);   
        mecCenterPub_ = this->nh_.advertise<geometry_msgs::PointStamped>("minimum_enclosing_circle_center",10,true);
        transportedObjectPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("transported_object_corners", 10, true);


        if(this->service_flag_)
        {
            this->mecService_ = this->nh_.advertiseService("mec_Info", &FormationLayer::mecInfoServiceCallback, this);
            this->service_flag_ = false;
        }


        //get the number of robots from the launch file
        std::string robots_number_key;
        if(this->nh_.searchParam("robots_number", robots_number_key))
        {
			this->nh_.getParam(robots_number_key, robots_number_);
            ROS_INFO("Number of Robots is:%d", robots_number_);
            

            for (int i = 0; i < robots_number_; i++)
            {

                //position topic
                std::string topic_name = "/robot" + std::to_string(i) + "/amcl_pose";

                //robot ID
                string robot_id = "robot_" + to_string(i);
                
                //create and save Callback functions 
                posesCallbacks_.push_back ([this, i, robot_id](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
                    this->robot_positions_[robot_id] = *msg;
                }); 

                //create and save subscribers 
                posesSubscribers_.push_back(this->nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 10, posesCallbacks_[i]));

            }
        }
		else
			ROS_ERROR("No Robots number parameter was found in the launch file.");

        dsrv_ = NULL;
        setupDynamicReconfigure(nh_);

        ROS_INFO("FormationLayer::onInitialize END");      
    }

    void FormationLayer::setupDynamicReconfigure(ros::NodeHandle& nh_)
    {
        dsrv_ = new dynamic_reconfigure::Server<formation_layer::FormationLayerConfig>(nh_);
        dynamic_reconfigure::Server<formation_layer::FormationLayerConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);
    }

    FormationLayer::~FormationLayer(){
    if (dsrv_)
        delete dsrv_;
    } 

    void FormationLayer::reconfigureCB(formation_layer::FormationLayerConfig &config, uint32_t level)
    {
        enabled_ = config.formation_layer_enabled;
        
        this->formation_properties_ = config.formation_properties_generated;

        this->transported_object_corners_ = polygon();
        this->transport_object_ = config.transport_object_included;
        this->corner1_.x = config.corner_point_1_x;
        this->corner1_.y = config.corner_point_1_y;
        this->transported_object_corners_.push_back(this->corner1_);
        this->corner2_.x = config.corner_point_2_x;
        this->corner2_.y = config.corner_point_2_y;
        this->transported_object_corners_.push_back(this->corner2_);
        this->corner3_.x = config.corner_point_3_x;
        this->corner3_.y = config.corner_point_3_y;
        this->transported_object_corners_.push_back(this->corner3_);
        this->corner4_.x = config.corner_point_4_x;
        this->corner4_.y = config.corner_point_4_y;
        this->transported_object_corners_.push_back(this->corner4_);
        this->corner5_.x = config.corner_point_5_x;
        this->corner5_.y = config.corner_point_5_y;
        this->transported_object_corners_.push_back(this->corner5_);

    } 

    void FormationLayer::getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, Polygon &RobotFootprint)
    {
        geometry_msgs::PolygonStamped RobotFootprintStamped;
        geometry_msgs::Quaternion orientation = position.pose.pose.orientation;
        tf::Quaternion tf_quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        double yaw = tf::getYaw(tf_quaternion);
        //build the oriented footprint 
        costmap_2d::transformFootprint(position.pose.pose.position.x, position.pose.pose.position.y,
                        yaw, getFootprint(), RobotFootprintStamped );
        RobotFootprint.clear();
        for(const auto& point : RobotFootprintStamped.polygon.points)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            RobotFootprint.push_back(p);  
        }
    }   

    Polygon FormationLayer::calculateBoundingBox(Polygon &footprint)
    {
        Polygon boundingbox;
        double min_x = footprint[0].x;
        double min_y = footprint[0].y;
        double max_x = footprint[0].x;
        double max_y = footprint[0].y;
        
        for(int i = 0; i < footprint.size(); i++)
        {
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

    void FormationLayer::reconfigureInflationRadius(double radius)
    {
        fpp_msgs::DynReconfigure::Request req;
        req.new_inflation_radius = radius;
        req.robot_namespace = "/robot0";

        dynamic_reconfigure::DoubleParameter inflation_reconfig;
        inflation_reconfig.name = "inflation_radius";
        inflation_reconfig.value = req.new_inflation_radius;
        dynamic_reconfigure::Reconfigure dyn_reconfigure_inflation_msg;
        dyn_reconfigure_inflation_msg.request.config.doubles.push_back(inflation_reconfig);
        
        ros::service::call(req.robot_namespace + "/move_base_flex/global_costmap/inflation/set_parameters", 
                            dyn_reconfigure_inflation_msg.request, 
                            dyn_reconfigure_inflation_msg.response);
    }

    bool FormationLayer::mecInfoServiceCallback(fpp_msgs::FormationFootprintInfo::Request &req, fpp_msgs::FormationFootprintInfo::Response &res)
    {
        res.formation_centre.x = this->mec_.C.X;
        res.formation_centre.y = this->mec_.C.Y;
        res.formation_centre.theta = 0.0;
        res.minimal_encloring_circle_radius = this->mec_.R;
        return true;
    }

    void FormationLayer::transformToMapFrame(Polygon &corners, vector<geometry_msgs::PointStamped> &transformed_corners)
    {
        
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        
        for(int i = 0; i<corners.size(); ++i)
        {
            
            try
            {   
                geometry_msgs::PointStamped corner;
                corner.header.frame_id = "robot0/base_link";
                corner.point.x = corners[i].x;
                corner.point.y = corners[i].y;
                geometry_msgs::PointStamped transformed_corner;
                tfBuffer.transform(corner, transformed_corner, "map");
                transformed_corners.push_back(transformed_corner);
            }
            catch(tf2::TransformException& ex)
            {
                ROS_WARN_STREAM("Failed to transform corner point "<< i);
            } 
        }
    }

    void FormationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                        double* max_x, double* max_y)
    {
        if(!enabled_)
            return;

        if(robot_positions_.size() == robots_number_){
            this->footprint_points_ = vector<geometry_msgs::Point>();
            this->formation_fp_points_ = vector<Point>();

            for(int i = 0 ; i < robot_positions_.size(); i++)
            { 
                string robot_id = "robot_" + to_string(i);

                getUnitFootprint(robot_positions_[robot_id], this->footprints_[robot_id]);

                for(const auto& point : footprints_[robot_id])
                {
                    this->footprint_points_.push_back(point);
                }
            
            }
            
          
            if(!this->transport_object_) //formation footprint without the transported object corners
            {
                this->formation_footprint_ = findConvexHull(this->footprint_points_, this->footprint_points_.size());

                //publish formation footprint to visualize
                geometry_msgs::PolygonStamped formation_footprint_msg;
                formation_footprint_msg.header.frame_id = "map";
                formation_footprint_msg.header.stamp = ros::Time::now();
                
                //publish formation footprint to visualize
                for(const auto& point : this->formation_footprint_)
                {
                    geometry_msgs::Point32 p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0;
                    formation_footprint_msg.polygon.points.push_back(p);
                }
                formationFPPub_.publish(formation_footprint_msg);
            }

            else                      //formation footprint with the transported object corners
            {
                this->transformed_object_corners_ = vector<geometry_msgs::PointStamped>();
                transformToMapFrame(this->transported_object_corners_, this->transformed_object_corners_);
                
                for(int i = 0; i<this->transformed_object_corners_.size(); ++i)
                {
                    geometry_msgs::Point p;
                    p.x = this->transformed_object_corners_[i].point.x;
                    p.y = this->transformed_object_corners_[i].point.y;
                    this->footprint_points_.push_back(p);
                }
                this->formation_footprint_ = findConvexHull(this->footprint_points_, this->footprint_points_.size());
                
                if(this->transport_object_)
                {   
                    //publish formation footprint to visualize
                    geometry_msgs::PolygonStamped formation_footprint_msg;
                    formation_footprint_msg.header.frame_id = "map";
                    formation_footprint_msg.header.stamp = ros::Time::now();
                    for(const auto& point : this->formation_footprint_)
                    {
                        geometry_msgs::Point32 p;
                        p.x = point.x;
                        p.y = point.y;
                        p.z = 0;
                        formation_footprint_msg.polygon.points.push_back(p);
                    }
                    formationFPPub_.publish(formation_footprint_msg);
                    
                    //publish the transported objecr corners to visualize
                    visualization_msgs::MarkerArray corners_marker_array;
                    for (unsigned int i = 0; i < this->transformed_object_corners_.size(); ++i)
                    {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "map";  
                        marker.header.stamp = ros::Time::now();
                        marker.ns = "corners_marks";
                        marker.id = i;
                        marker.type = visualization_msgs::Marker::SPHERE;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position = this->transformed_object_corners_[i].point;  
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.07;  
                        marker.scale.y = 0.07;
                        marker.scale.z = 0.07;
                        marker.color.r = 0.0; 
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        marker.color.a = 1.0;
                        corners_marker_array.markers.push_back(marker);
                    }
                    this->transportedObjectPub_.publish(corners_marker_array);
                }
            }

            //get the BoundingBox of the formation footprint
            this->bounding_box_ = vector<geometry_msgs::Point>();
            this->bounding_box_ = calculateBoundingBox(formation_footprint_);
            *min_x = std::min(bounding_box_[2].x, *min_x); 
            *min_y = std::min(bounding_box_[2].y, *min_y);
            *max_x = std::max(bounding_box_[0].x, *max_x);
            *max_y = std::max(bounding_box_[0].y, *max_y);
            
            //publish the bouding box to visualize
            geometry_msgs::PolygonStamped boundingBoxMsg;
            boundingBoxMsg.header.frame_id = "map";
            boundingBoxMsg.header.stamp = ros::Time::now();
            
            for(const auto& point : this->bounding_box_)
            {
                geometry_msgs::Point32 p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0;
                boundingBoxMsg.polygon.points.push_back(p);
            }
            boundingBoxPub_.publish(boundingBoxMsg);
            
            if(this->formation_properties_){
                
                //get the minimum enclosing circle of the formation          
                for(const auto& point : this->formation_footprint_)
                {
                    Point p;
                    p.X = point.x;
                    p.Y = point.y;
                    this->formation_fp_points_.push_back(p);
                }
                this->mec_ = welzl(formation_fp_points_);
                
                //publish the formation minimum enclosing circle center to visualize
                geometry_msgs::PointStamped mecC_msg;
                mecC_msg.header.frame_id = "map";
                mecC_msg.point.x = mec_.C.X;
                mecC_msg.point.y = mec_.C.Y;
                mecC_msg.point.z = 0.0;
                this->mecCenterPub_.publish(mecC_msg);

                reconfigureInflationRadius(mec_.R * 3.0); // reconfigure the inflation radius with safety factor 3.0                                           

                // publish the formaiton minimum ecnlosing circle to visualize
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = mec_.C.X;
                marker.pose.position.y = mec_.C.Y;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 2.0 * mec_.R;
                marker.scale.y = 2.0 * mec_.R;
                marker.scale.z = 0.1;
                marker.color.a = 0.59;
                marker.color.r = 1.0;
                marker.color.g = 0.6;
                marker.color.b = 0.6;
                this->mecPub_.publish(marker);
            }
        }
    }

    void FormationLayer::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
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

        for (; n > 0; --n) 
        {
            cells.push_back(pt);

            if (error > 0) 
            {
                pt.x += x_inc;
                error -= dy;
            } 
            
            else 
            {
                pt.y += y_inc;
                error += dx;
            }
        }
    }

    void FormationLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
    {

        for (unsigned int i = 0; i < polygon.size() - 1; ++i) 
        {
            linetrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
        }

        if (!polygon.empty()) 
        {
            unsigned int last_index = polygon.size() - 1;
            linetrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
        }
    }

    void FormationLayer::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
    {

        if (polygon.size() < 3)
            return;

        //first get the cells that make up the outline of the polygon
        polygonOutlineCells(polygon, polygon_cells);

        if (!fill)
            return;

        //sorting points by x
        PointInt swap;
        unsigned int i = 0;
        while (i < polygon_cells.size() - 1) 
        {
            
            if (polygon_cells[i].x > polygon_cells[i + 1].x) 
            {
                swap = polygon_cells[i];
                polygon_cells[i] = polygon_cells[i + 1];
                polygon_cells[i + 1] = swap;

                if (i > 0)
                    --i;
            } 
            
            else
                ++i;
        }

        i = 0;
        PointInt min_pt;
        PointInt max_pt;
        int min_x = polygon_cells[0].x;
        int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

        //walk through each column and mark cells inside the polygon
        for (int x = min_x; x <= max_x; ++x) 
        {
            if (i >= (int)polygon_cells.size() - 1)
                break;

            if (polygon_cells[i].y < polygon_cells[i + 1].y) 
            {
                min_pt = polygon_cells[i];
                max_pt = polygon_cells[i + 1];
            } 
            else 
            {
                min_pt = polygon_cells[i + 1];
                max_pt = polygon_cells[i];
            }

            i += 2;
            while (i < polygon_cells.size() && polygon_cells[i].x == x) 
            {
                if (polygon_cells[i].y < min_pt.y)
                    min_pt = polygon_cells[i];
                else if (polygon_cells[i].y > max_pt.y)
                    max_pt = polygon_cells[i];
                ++i;
            }

            PointInt pt;

            for (int y = min_pt.y; y < max_pt.y; ++y) 
            {
                pt.x = x;
                pt.y = y;
                polygon_cells.push_back(pt);
            }
        }
    }

    void FormationLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon,
                        unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
    {   
        std::vector<PointInt> map_polygon;
        
        for (unsigned int i = 0; i < polygon.size(); ++i) 
        {
            PointInt loc;
            master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
            map_polygon.push_back(loc);
        }

        std::vector<PointInt> polygon_cells;

        // get the cells that fill the polygon
        rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

        // set the cost of those cells
        for (unsigned int i = 0; i < polygon_cells.size(); ++i) 
        {  
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

    void FormationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if(!enabled_)
            return;

        if(this->footprints_.size() == robots_number_)
        {

            for(int i = 0 ; i < robots_number_; i++)
            {
                string robot_id = "robot_" + to_string(i);
                setPolygonCost(master_grid, this->footprints_[robot_id], FREE_SPACE, min_i, min_j, max_i, max_j, true);
            }
            
        }

    }

} //end_namespace formation_layer
            