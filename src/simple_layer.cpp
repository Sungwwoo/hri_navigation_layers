#include <hri_navigation_layers/simple_layer.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <list>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <string>


PLUGINLIB_EXPORT_CLASS(hri_navigation_layers::SimpleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace hri_navigation_layers{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_), g_nh;
    ROS_INFO("Initializing SimpleLayer Plugin...");
    sub_point_ = nh.subscribe("/clicked_point", 1, &SimpleLayer::cbPoint, this);
    first_time_ = true;
    enabled_ = true;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y){
    
    // boost::recursive_mutex::scoped_lock lock(lock_);
    std::string global_frame = layered_costmap_->getGlobalFrameID();


    if (first_time_){
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        first_time_ = false;
    }
    else{
        double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
        *min_x = std::min(last_min_x_, *min_x);
        *min_y = std::min(last_min_y_, *min_y);
        *max_x = std::max(last_max_x_, *max_x);
        *max_y = std::max(last_max_y_, *max_y);
        last_min_x_ = a;
        last_min_y_ = b;
        last_max_x_ = c;
        last_max_y_ = d;
    }
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
    
    // boost::recursive_mutex::scoped_lock lock(lock_);
    ROS_INFO("Point array has length %lu", transformedPoints_.size());
    if (transformedPoints_.empty()) return;

    // Get costmap
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

    double res = costmap->getResolution();
    ROS_INFO("Updating costs...");
    geometry_msgs::PointStamped point = transformedPoints_.back();
    int cx = static_cast<int>(point.point.x / res), cy = static_cast<int>(point.point.y / res);
    int radius = static_cast<int>(size_/2 / res);
    int max_x = static_cast<int>(costmap->getSizeInCellsX()), max_y = static_cast<int>(costmap->getSizeInCellsY());

    int start_x = cx - radius, start_y = cy - radius, end_x = cx + radius, end_y = cy + radius;

    if (start_x < 0) start_x = 0;
    if (start_y < 0) start_y = 0;
    if (end_x > max_x) end_x = max_x;
    if (end_y > max_y) end_y = max_y;

    for (int i = start_x; i < end_x; i++)
        for (int j = start_y; j < end_y; j++)
            if ((cx - i)*(cx - i) + (cy - j)*(cy - j) > radius*radius)
                costmap->setCost(i, j, costmap_2d::FREE_SPACE);
            else
                costmap->setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
        
    
}

void SimpleLayer::cbPoint(const geometry_msgs::PointStamped& point){

    // boost::recursive_mutex::scoped_lock lock(lock_);
    ROS_INFO("Got new point");

    std::string global_frame = layered_costmap_->getGlobalFrameID();
    geometry_msgs::PointStamped in, out;

    if (point.header.frame_id != global_frame){
        in.header.frame_id = point.header.frame_id;
        in.header.stamp = point.header.stamp;
        in.point = point.point;

        tf_->transform(in, out, global_frame);

        transformedPoints_.push_back(out);        
        ROS_INFO("Transformed point from %s frame to %s frame", point.header.frame_id.c_str(), global_frame.c_str());
    }
    else{
        transformedPoints_.push_back(point);
    }

}

void SimpleLayer::reconfigure(SimpleLayerConfig& config, uint32_t level){
    // enabled_ = config.enabled;
    size_ = config.size;
}

}