#ifndef FORMATION_LAYER_H_
#define FORMATION_LAYER_H_
#include <formation_layer/FormationLayerConfig.h>
#include <formation_layer/convex_hull.cpp>
#include <formation_layer/enclosing_circle.cpp>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/costmap_layer.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <vector>
#include <string>
#include <iostream>
#include <functional>
#include <map>
#include <cmath>

#include <fpp_msgs/DynReconfigure.h>
#include <fpp_msgs/FormationFootprintInfo.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/client.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using Polygon = vector<geometry_msgs::Point>;

namespace formation_layer_namespace
{
    struct PointInt {
        int x;
        int y;
    };

    class FormationLayer : public costmap_2d::Layer
    {
    public :

        FormationLayer();
        virtual ~FormationLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected : 

        virtual void setupDynamicReconfigure(ros::NodeHandle& nh_);

    private :

        ros::NodeHandle nh_;
        
        ros::Subscriber formationFPSubs_;
        
        ros::Publisher formationFPPub_;
        ros::Publisher footprintsPub_;
        ros::Publisher mecPub_;
        ros::Publisher boundingBoxPub_;
        ros::Publisher InflationRadiusPub_;
        ros::Publisher mecCenterPub_;
        ros::Publisher transportedObjectPub_;
        ros::Publisher markerPub_;

        ros::ServiceServer mecService_;

        int robots_number_;

        dynamic_reconfigure::Server<formation_layer::FormationLayerConfig> *dsrv_;

        vector<std::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>> posesCallbacks_;

        bool transport_object_;
        bool formation_properties_;

        Polygon transported_object_corners_;
        vector<geometry_msgs::PointStamped> transformed_object_corners_;
        geometry_msgs::Point corner1_;
        geometry_msgs::Point corner2_;
        geometry_msgs::Point corner3_;
        geometry_msgs::Point corner4_;
        geometry_msgs::Point corner5_;


        bool service_flag_ = true; //this flag is used to advertise the mec service only once

        vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses_;

        map<std::string, geometry_msgs::PoseWithCovarianceStamped> robot_positions_;

        vector<ros::Subscriber> posesSubscribers_;

        Polygon footprint_;

        map<std::string, Polygon> footprints_;


        Polygon footprint_points_;

        Polygon formation_footprint_;

        Polygon bounding_box_;

        vector<Point> formation_fp_points_;

        Circle mec_;

        void reconfigureCB(formation_layer::FormationLayerConfig &config, uint32_t level);

        /// \brief             calculate Unit Footprint in the Map frame
        /// \param position    position of the Unit
        void getUnitFootprint(const geometry_msgs::PoseWithCovarianceStamped &position, Polygon &RobotFootprint);

        /// \brief             rasterizes line between two map coordinates into a set of cells
        /// \param x0          line start x-coordinate (map frame)
        /// \param y0          line start y-coordinate (map frame)
        /// \param x1          line end x-coordinate (map frame)
        /// \param y1          line end y-coordinate (map frame)
        /// \param[out] cells  new cells in map coordinates are pushed back on this container
        void linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells);

        /// \brief                     extracts the boundary of a polygon in terms of map cells
        /// \note                      this method is based on Costmap2D::polygonOutlineCells() but accounts for a self - implemented 
        ///                            raytrace algorithm and allows negative map coordinates
        /// \param polygon             polygon defined  by a vector of map coordinates
        /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
        void polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells);

        /// \brief                     converts polygon (in map coordinates) to a set of cells in the map
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

        /// @brief          dynamic reconfigure the inflation radius
        /// @param radius   radius of the inflation area 
        void reconfigureInflationRadius(double radius);

        /// @brief          Callback function for mecService
        bool mecInfoServiceCallback(fpp_msgs::FormationFootprintInfo::Request &req, fpp_msgs::FormationFootprintInfo::Response &res);
        
        /// @brief          transform the corner points of the transported object to the map frame
        void transformToMapFrame(Polygon &corners, vector<geometry_msgs::PointStamped> &transformed_corners);

        /// @brief          calculate the bounding box of the formation footprint for the updateBounds function
        Polygon calculateBoundingBox(Polygon &footprint);

    };

} //end_namespace formation_layer
#endif //FORMATION_LAYER_H_