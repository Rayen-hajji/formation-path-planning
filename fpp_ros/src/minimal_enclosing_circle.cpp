#include <fpp_ros/path_planner/minimal_enclosing_circle.h>

namespace fpp_helper
{
    MinimalEnclosingCircle::MinimalEnclosingCircle()
    {

    }

    void MinimalEnclosingCircle::calcMinimalEnclosingCircle(std::vector<Eigen::Vector2d> points_to_enclose)
    {
        this->enclosed_points_.push_back(points_to_enclose.back());
        this->circle_defining_points_.push_back(points_to_enclose.back());
        points_to_enclose.pop_back();
        this->enclosed_points_.push_back(points_to_enclose.back());
        this->circle_defining_points_.push_back(points_to_enclose.back());
        points_to_enclose.pop_back();

        // Initialize minimal circle with two points
        this->updateCircle();

        if(points_to_enclose.size() != 0)
        {
            for(Eigen::Vector2d point: points_to_enclose)
            {
                if(std::find(this->enclosed_points_.begin(), this->enclosed_points_.end(), point) == this->enclosed_points_.end())
                {
                    this->enclosed_points_.push_back(point);
                }

                double distance_to_centre = this->calcDistance(this->circle_centre_, point);
                if(distance_to_centre > this->circle_radius_)
                {
                    this->circle_defining_points_.push_back(point);
                    this->findNewSmallestCircle();
                }
            }
        }
    }

    void MinimalEnclosingCircle::findNewSmallestCircle()
    {
        if(this->circle_defining_points_.size() == 3)
        {
            for(Eigen::Vector2d outer_point: this->circle_defining_points_)
            {
                Eigen::Vector2d diff_vector1;
                diff_vector1 << 0, 0;
                Eigen::Vector2d diff_vector2;
                diff_vector2 << 0, 0;
                for(Eigen::Vector2d inner_point: this->circle_defining_points_)
                {
                    if(outer_point != inner_point)
                    {
                        if(diff_vector1[0] == 0 && diff_vector1[1] == 0)
                        {
                            diff_vector1 = inner_point - outer_point;
                        }
                        else
                        {
                            diff_vector2 = inner_point - outer_point;
                        }
                    }
                }
                Eigen::Vector2d diff_vector = diff_vector1 - diff_vector2;
                double angle = atan2(diff_vector[1], diff_vector[0]);

                if(angle > M_PI_2) // If angle is bigger than 90° than it is an obtuse angle
                {
                    std::vector<Eigen::Vector2d>::iterator point_to_delete = std::find(this->circle_defining_points_.begin(),
                                                                                       this->circle_defining_points_.end(),
                                                                                       outer_point);
                    this->circle_defining_points_.erase(point_to_delete);
                    break;
                }
            }
            this->updateCircle();
        }
        else if(this->circle_defining_points_.size() == 4)
        {
            Eigen::Vector2d new_point = this->circle_defining_points_.back();
            
            std::vector<Eigen::Vector2d> best_circle_defining_points;
            double smallest_circle_radius;

            // Init
            smallest_circle_radius = this->calcDistance(new_point, this->circle_defining_points_[0]);

            // First try forming smallest circle with new point and one of the old points
            for(Eigen::Vector2d point: this->circle_defining_points_)
            {
                if(point != new_point)
                {
                    
                }
            }

            // Then try forming smallest circle with new point and two of the old points
        }
    }

    void MinimalEnclosingCircle::updateCircle()
    {
        if(this->circle_defining_points_.size() == 2)
        {
            this->circle_centre_ = this->calcCentreOfVector(this->enclosed_points_[0], this->enclosed_points_[1]);
            this->circle_radius_ = 0.5 * this->calcDistance(this->enclosed_points_[0], this->enclosed_points_[1]);
        }
        else if(this->circle_defining_points_.size() == 3)
        {
            Eigen::Vector2d vector_centre1 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[0]);
            Eigen::Vector2d orthogonal_vector1 = this->calcOrthogonalVector(this->circle_defining_points_[0] - this->circle_defining_points_[1]);
            Eigen::Vector2d vector_centre2 = this->calcCentreOfVector(this->circle_defining_points_[1], this->circle_defining_points_[2]);
            Eigen::Vector2d orthogonal_vector2 = this->calcOrthogonalVector(this->circle_defining_points_[2] - this->circle_defining_points_[1]);

            this->circle_centre_ = this->calcVectorLineIntersectionPoint(vector_centre1, orthogonal_vector1,
                                                                         vector_centre2, orthogonal_vector2);
            this->circle_radius_ = this->calcDistance(this->circle_defining_points_[0], this->circle_centre_);
        }
        else
        {
            std::cout << "MinimalEnclosingCircle: Error during updateCircle method. Number of points define circle: " << std::to_string(this->circle_defining_points_.size());
        }
    }

    Eigen::Vector2d MinimalEnclosingCircle::calcCentreOfVector(Eigen::Vector2d first_point, Eigen::Vector2d second_point)
    {
        Eigen::Vector2d diff_vector = second_point - first_point;
        Eigen::Vector2d relative_circle_centre = 0.5 * diff_vector;
        return first_point + relative_circle_centre;
    }

    Eigen::Vector2d MinimalEnclosingCircle::calcCentreOfVector(Eigen::Vector2d start_point, Eigen::Vector2d vector_to_end_point)
    {
        return start_point + 0.5 * vector_to_end_point;
    }

    Eigen::Vector2d MinimalEnclosingCircle::calcOrthogonalVector(Eigen::Vector2d vector)
    {
        // Calculating the orthogonal vector is just swaping x and y and after that inverting the x coordinate
        Eigen::Vector2d orthognal_vector;
        orthognal_vector[0] = -vector[1];
        orthognal_vector[1] = vector[0];
        return orthognal_vector;
    }

    Eigen::Vector2d MinimalEnclosingCircle::calcVectorLineIntersectionPoint(Eigen::Vector2d lead_vector1,
                                                                            Eigen::Vector2d direction_vector1,
                                                                            Eigen::Vector2d lead_vector2,
                                                                            Eigen::Vector2d direction_vector2)
    {
        double numerator = ((lead_vector2[1] * direction_vector2[0]) + (lead_vector1[0] * direction_vector2[1]) -
                            (lead_vector2[0] * direction_vector2[1]) - (lead_vector1[1] * direction_vector2[0]));

        double denominator = (direction_vector1[1] * direction_vector2[0]) - (direction_vector1[0] * direction_vector2[1]);

        double factor = numerator / denominator;

        return lead_vector1 + (factor * direction_vector1);
    }

    double MinimalEnclosingCircle::calcDistance(Eigen::Vector2d first_point, Eigen::Vector2d second_point)
    {
        Eigen::Vector2d diff_vector = second_point - first_point;
        return diff_vector.norm();
    }
}