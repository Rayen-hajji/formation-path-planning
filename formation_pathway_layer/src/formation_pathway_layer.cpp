#include <formation_pathway_layer/formation_pathway_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(formation_pathway_layer_namespace::FormationPathwayLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
namespace formation_pathway_layer_namespace
{
    FormationPathwayLayer::FormationPathwayLayer() {} 
    
    void FormationPathwayLayer::onInitialize()
    {
        ROS_INFO("FormationPathwayLayer::onInitialize");
        this->nh_ = ros::NodeHandle("~/"+ name_);
        rolling_window_ = layered_costmap_->isRolling();

        FormationPathwayLayer::matchSize();
        current_= true;

        //get the robots number from launch file
        std::string robots_number_key;
        if(this->nh_.searchParam("robots_number", robots_number_key))
			this->nh_.getParam(robots_number_key, robots_number);
        else {
            ROS_ERROR("FormationPathwayLayer::onInitialize: robots_number not found in the Launch File");
            return;
        }

        //get the initial y_positions of the robots
        this->y_positions = vector<double>();
        for(int i = 0 ; i < robots_number ; i++){
            std:: string argument_name = "robot" + std::to_string(i) + "_y_pos";
            ROS_INFO("parameter name %s", argument_name.c_str());
            std::string y_position_key;
            double position;
            if(this->nh_.searchParam(argument_name, y_position_key)){
                this->nh_.getParam(y_position_key, position);
                this->y_positions.push_back(position);
            }
            else {
                ROS_ERROR("FormationPathwayLayer::onInitialize: %s not found in the Launch File", argument_name.c_str());
                return;
            }
        }

        // formationFPSubs_ = this->nh_.subscribe("/robot0/move_base_flex/formation_footprint",10, &FormationPathwayLayer::formationFPCallback, this);
        
        //Plugin initialization
        dsrv_ = NULL;
        setupDynamicReconfigure(nh_);
        ROS_INFO("FormationPathwayLayer::onInitialize END");
    }

    void FormationPathwayLayer::setupDynamicReconfigure(ros::NodeHandle& nh_){
        dsrv_ = new dynamic_reconfigure::Server<formation_pathway_layer::FormationPathwayLayerConfig>(nh_);
        dynamic_reconfigure::Server<formation_pathway_layer::FormationPathwayLayerConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);
    }

    FormationPathwayLayer::~FormationPathwayLayer(){
    if (dsrv_)
        delete dsrv_;
    } 

    void FormationPathwayLayer::reconfigureCB(formation_pathway_layer::FormationPathwayLayerConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }      

    // void FormationPathwayLayer::formationFPCallback(const geometry_msgs::PolygonStamped &msg)
    // {
    //     ROS_INFO("Formation Callback started");
    //     formation_fp.clear();
    //     for(const auto& point : msg.polygon.points){
    //         geometry_msgs::Point p;
    //         p.x = point.x;
    //         p.y = point.y;
    //         p.z = 0;
    //         formation_fp.push_back(p);     
    //     }
    //     for(int i = 0; i < formation_fp.size(); ++i){
    //         ROS_INFO("FP point[%d]={%f,%f}", i, formation_fp[i].x, formation_fp[i].y);
    //     } 
    //     ROS_INFO("Formation Callback done");
    // }

    double FormationPathwayLayer::calculaterange(std::vector<double> positions, double safety_distance){
        double max = positions[0];
        double min = positions[0];
        for (int i=0 ; i < positions.size() ; i++){
            if (positions[i] > max){
                max = positions[i];
            }
            if (positions[i] < min){
                min = positions[i];
            }
        }
        return max - min + 0.64 + (2*safety_distance);
    }
    

    void FormationPathwayLayer::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
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

    void FormationPathwayLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
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

    void FormationPathwayLayer::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
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
    
    void FormationPathwayLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
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

    void FormationPathwayLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon,
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
    void FormationPathwayLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        ROS_INFO("UpdateCosts started");
        if(!enabled_)
            return;
        for(int i = 0; i < 3; i++)
        ROS_INFO("y_position of robot %d is %f",i,y_positions[i]);
        
        if(!y_positions.empty()){
            formation_width = calculaterange(this->y_positions, 0.3);
        }
        // formation_width = calculaterange(this->y_positions);
        ROS_INFO("formstion width = %f",formation_width);
        master_grid.worldToMapNoBounds(-3.2,-2.65,first_path_point.x,first_path_point.y);
        master_grid.worldToMapNoBounds(3.30,-2.68,last_path_point.x,last_path_point.y);
        linetrace(first_path_point.x, first_path_point.y, last_path_point.x, last_path_point.y, path);
        for(int i = 0 ; i < path.size() ; i++){
            master_grid.setCost(path[i].x,path[i].y,FREE_SPACE);
        }
    }
}//end_namespace