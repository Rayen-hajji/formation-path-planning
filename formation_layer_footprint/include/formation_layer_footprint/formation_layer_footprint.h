#ifndef FORMATION_LAYER_Footprint_H_
#define FORMATION_LAYER_Footprint_H_
#include <formation_layer_footprint/FormationLayerFootprintConfig.h>
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
#include <vector>
#include <string>
#include <iostream>
#include <functional>

using namespace std;
using Polygon = std::vector<geometry_msgs::Point32>;
using FootprintPolygon = std::vector<geometry_msgs::PolygonStamped>;

namespace formation_layer_footprint_namespace
{
struct PointInt {
    int x;
    int y;
};
class FormationLayerFootprint : public costmap_2d::Layer
{
public :
    
    FormationLayerFootprint();
    virtual ~FormationLayerFootprint();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    // virtual void activate();
    // virtual void deactivate();
    // virtual void reset();

protected : 
    virtual void setupDynamicReconfigure(ros::NodeHandle& nh_);
    // bool rolling_window_;

private :
    int robots_number;
    
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<formation_layer_footprint::FormationLayerFootprintConfig> *dsrv_;

    //vector to save the Callback functions
    vector<std::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>> callbacks;

    double mark_x_, mark_y_;

    //polygon to save a footprint  
    Polygon RobotFootprint_0, RobotFootprint_1, RobotFootprint_2;
    geometry_msgs::PolygonStamped RobotFootprintStamped_0, RobotFootprintStamped_1, RobotFootprintStamped_2;

    //vector to save all units poses
    vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses;
    
    //vector to save all subscribers
    vector<ros::Subscriber> Subscribers;

    //vector to save all units footprints
    vector<Polygon> robots_footprints;

    // Variable to save a position 
    geometry_msgs::PoseWithCovarianceStamped RobotPose_0, RobotPose_1, RobotPose_2;

    //Vector to save the footprints of participated Robots
    FootprintPolygon RobotsFootprints;
    

    void reconfigureCB(formation_layer_footprint::FormationLayerFootprintConfig &config, uint32_t level);

    /// \brief      create callback functions for the units positions 
    ///  \param n   number of the units 
    void createCallbacks(int n);

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    
    //callback function of the robot position 
    // void poseCallback_0(const geometry_msgs::PoseWithCovarianceStamped &msg);
    // void poseCallback_1(const geometry_msgs::PoseWithCovarianceStamped &msg);
    // void poseCallback_2(const geometry_msgs::PoseWithCovarianceStamped &msg);

    /// \brief             calculate Unit Footprint in the Map frame
    /// \param position    position of the Unit
    void getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, Polygon RobotFootprint);
    // void getUnitFootprint_0(const geometry_msgs::PoseWithCovarianceStamped &position);
    // void getUnitFootprint_1(const geometry_msgs::PoseWithCovarianceStamped &position);
    // void getUnitFootprint_2(const geometry_msgs::PoseWithCovarianceStamped &position);

    /// \brief             rasterizes line between two map coordinates into a set of cells
    /// \note              since Costmap2D::raytraceLine() is based on the size_x and since we want to rasterize polygons that might also be located outside map bounds we provide a modified raytrace
    ///                    implementation(also Bresenham) based on the integer version presented here : http : //playtechs.blogspot.de/2007/03/raytracing-on-grid.html
    /// \param x0          line start x-coordinate (map frame)
    /// \param y0          line start y-coordinate (map frame)
    /// \param x1          line end x-coordinate (map frame)
    /// \param y1          line end y-coordinate (map frame)
    /// \param[out] cells  new cells in map coordinates are pushed back on this container
    void linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells);
    
    /// \brief                     extracts the boundary of a polygon in terms of map cells
    /// \note                      this method is based on Costmap2D::polygonOutlineCells() but accounts for a self - implemented raytrace algorithm and allows negative map coordinates
    /// \param polygon             polygon defined  by a vector of map coordinates
    /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
    void polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells);
    
    /// \brief                     converts polygon (in map coordinates) to a set of cells in the map
    /// \note                      This method is mainly based on Costmap2D::convexFillCells() but accounts for a self - implemented polygonOutlineCells() method and allows negative map coordinates
    /// \param polygon             Polygon defined  by a vector of map coordinates
    /// \param fill                If true, the interior of the polygon will be considered as well
    /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
    void rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill);

    /// \brief                set cost in a Costmap2D for a polygon (polygon may be located outside bounds)
    /// \param master_grid    reference to the Costmap2D object
    /// \param polygon        polygon defined by a vector of points (in world coordinates)
    /// \param cost           the cost value to be set (0,255)
    /// \param min_i          minimum bound on the horizontal map index/coordinate
    /// \param min_j          minimum bound on the vertical map index/coordinate
    /// \param max_i          maximum bound on the horizontal map index/coordinate
    /// \param max_j          maximum bound on the vertical map index/coordinate
    /// \param fill_polygon   if true, tue cost for the interior of the polygon will be set as well
    void setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon,
                        unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);
    };
}
#endif