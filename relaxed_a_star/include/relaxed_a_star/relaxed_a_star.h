#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>

namespace relaxed_a_star
{
    /**
     * @brief Struct that acts as a container for storing the call with the according f_cost value
     * Contains overwriten > and < operators for multiset comparator functions
     * 
     */
    struct Cell
    {
        int cell_index;
        float f_cost;

        bool operator<(const Cell& rhs) const
        {
            return this->f_cost < rhs.f_cost;
        }

        bool operator>(const Cell& rhs) const
        {
            return this->f_cost > rhs.f_cost;
        }
    };

    enum NeighborType
    {
        FourWay = 4,
        EightWay = 8
    };

    class RelaxedAStar : public nav_core::BaseGlobalPlanner, public mbf_costmap_core::CostmapPlanner
    {
        public:
            RelaxedAStar();
            RelaxedAStar(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

            // mbf_costmap_core::CostmapPlanner interface implementation
            /**
            * @brief Initialization function for the CostmapPlanner
            * @param name The name of this planner
            * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
            */
            void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

            /**
             * @brief Initialization method for the CostmapPlanner
             * 
             * @param name The name of this planner
             * @param costmap A pointer to the costmap that will be used for planning
             * @param global_frame Global frame of the costmap
             */
            void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

            /**
             * @brief Given a goal pose in the world, compute a plan
             * @param start The start pose 
             * @param goal The goal pose 
             * @param plan The plan... filled by the planner
             * @return True if a valid plan was found, false otherwise
             */
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

            /**
             * @brief Given a goal pose in the world, compute a plan
             * @param start The start pose
             * @param goal The goal pose
             * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
             *        in x and y before failing
             * @param plan The plan... filled by the planner
             * @param cost The cost for the the plan
             * @param message Optional more detailed outcome as a string
             * @return Result code as described on GetPath action result:
             *         SUCCESS         = 0
             *         1..9 are reserved as plugin specific non-error results
             *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
             *         CANCELED        = 51
             *         INVALID_START   = 52
             *         INVALID_GOAL    = 53
             *         NO_PATH_FOUND   = 54
             *         PAT_EXCEEDED    = 55
             *         EMPTY_PATH      = 56
             *         TF_ERROR        = 57
             *         NOT_INITIALIZED = 58
             *         INVALID_PLUGIN  = 59
             *         INTERNAL_ERROR  = 60
             *         71..99 are reserved as plugin specific errors
             */
            uint32_t makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                        double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                        std::string &message);

            /**
            * @brief Requests the planner to cancel, e.g. if it takes too much time.
            * @remark New on MBF API
            * @return True if a cancel has been successfully requested, false if not implemented.
            */
            bool cancel();
        
        protected:

            bool initialized_;

            costmap_2d::Costmap2DROS *costmap_ros_;
            costmap_2d::Costmap2D *costmap_;
            std::shared_ptr<bool[]> occupancy_map_; // One dimensional represantation of the map. True = occupied, false = free

        private:
            float calcGCost(int current_g_cost, int current_cell_index, int target_cell_index);

            /**
             * @brief Calculates the heuristic cost from current position to the goal_cell.
             * Currently the cost will be approximated with the euclidean distance.
             * 
             * @param map_current_position Current cell from where the heuristic cost should be calculated to the goal
             * @param map_goal Goal of the path planning
             * @return float 
             */
            float calcHCost(int* map_current_position, int* map_goal);
            float calcHCost(int current_cell_index, int goal_cell_index);

            float calcFCost(float current_g_score, int current_cell_index, int goal_cell_index);


            /**
             * @brief Gets the index of the 2D position in the 1D representing array
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param map_point Array that contains x and y index of the costmap
             * @return int Cell index in one dimensional representation
             */
            int getArrayIndexByCostmapPoint(int *map_point);

            /**
             * @brief Gets the index of the 2D position in the 1D representing array
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param map_x_coord X coordinate of the point in the map as int
             * @param map_y_coord Y coordinate of the point in the map as int
             * @return int Cell index in one dimensional representation
             */
            int getArrayIndexByCostmapPoint(int map_x_coord, int map_y_coord);

            /**
             * @brief Gets the index of the 1D array in the 2D costmap
             * Here the costmap is represented by a 1 dimensional array.
             * 
             * @param array_index Index of the position in the one dimensional array
             * @param map_point Point in the two dimensional costmap
             */
            void getCostmapPointByArrayIndex(int array_index, int *map_point);

            /**
             * @brief Gets all free neighbor cells adjacent to the current cell
             * 
             * @param current_cell_index 
             * @return std::vector<int> 
             */
            std::vector<int> getFreeNeighborCells(int current_cell_index);

            bool isCellFree(int cell_index);

            float calcMoveCost(int current_cell_index, int target_cell_index);
            float calcMoveCost(int* map_current_position, int* map_target_position);
            float calcMoveCost(int map_current_pos_x, int map_current_pos_y, int map_target_pos_x, int map_target_pos_y);

            std::string global_frame_;
            std::string tf_prefix_;

            // Parameter list
            float default_tolerance_;
            NeighborType neighbor_type_;

            // Process information
            int map_size_;
    };
}
