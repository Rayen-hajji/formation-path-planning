#ifndef FORMATION_LAYER_Footprint_H_
#define FORMATION_LAYER_Footprint_H_
#include <formation_layer_footprint/FormationLayerFootprintConfig.h>
#include <formation_layer_footprint/fpl_utils.cpp>
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
using polygon = vector<geometry_msgs::Point>;

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

protected : 

    virtual void setupDynamicReconfigure(ros::NodeHandle& nh_);

private :
    
    ros::NodeHandle nh_;
    ros::Subscriber formationFPSubs;
    
    //preliminary solution to get the formation footprint OF THE Robot0
    ros::Subscriber robot0Subs;

    //formation footprint publisher
    ros::Publisher formationFPPub;
    
    dynamic_reconfigure::Server<formation_layer_footprint::FormationLayerFootprintConfig> *dsrv_;

    //vector to save the Callback functions
    vector<std::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>> callbacks;

    //points to mark the minimum and maximum of the formation footprint
    double mark_x_max, mark_y_max, mark_x_min, mark_y_min; 

    //variable to save the number of robots used in the formation 
    int robots_number;

    // vector to save all units positions
    vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses;
    
    // vector to save all ros subscribers
    vector<ros::Subscriber> Subscribers;

    // polygon to save a single footprint
    polygon footprint;

    // polygon to save all footprints
    std::vector<polygon> footprints;

    // polygon to save all footprints points
    polygon footprint_points;

    // polygon to save the subscribed formation footprint   
    polygon formation_fp;

    // polygon to save the calculated formation footprint
    polygon formation_footprint;

    void formationFPCallback(const geometry_msgs::PolygonStamped &msg);

    void reconfigureCB(formation_layer_footprint::FormationLayerFootprintConfig &config, uint32_t level);

    /// \brief             calculate Unit Footprint in the Map frame
    /// \param position    position of the Unit
    void getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, polygon &RobotFootprint, int index);

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
    void setPolygonCost(costmap_2d::Costmap2D &master_grid, const polygon &polygon,
                        unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);
    };
}
#endif