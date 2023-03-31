#include <hri_navigation_layers/simple_layer.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <list>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

    cb_ = boost::bind(&SimpleLayer::reconfigure, this, _1, _2);
    dsrv_ = new dynamic_reconfigure::Server<SimpleLayerConfig>(ros::NodeHandle("~/" + name_));
    dsrv_->setCallback(cb_);
    sub_point_ = nh.subscribe("/clicked_point", 1, &SimpleLayer::cbPoint, this);
    g_sub_point_ = g_nh.subscribe("/clicked_point", 1, &SimpleLayer::cbGPoint, this);
    pub_clicked_point_marker_ = nh.advertise<visualization_msgs::Marker>("/cost_point", 1);
    first_time_ = true;
    current_ = true;
    cost_ = static_cast<unsigned char>(250);
    
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y){
    
    boost::recursive_mutex::scoped_lock lock(lock_);
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
    
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (transformedPoints_.empty()) return;
    // Get costmap
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

    double res = costmap->getResolution();
    // ROS_INFO("Updating costs...");
    geometry_msgs::PointStamped point = transformedPoints_.back();
    visualization_msgs::Marker marker;
    marker.header = point.header;
    marker.id = 0;
    marker.action = 0;
    marker.type = 3;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.15;
    marker.color.b = 1.0;
    marker.color.a = 0.7;
    marker.lifetime = ros::Duration(0);

    pub_clicked_point_marker_.publish(marker);


    double cx = point.point.x, cy = point.point.y;
    int radius = static_cast<int>(size_/2/res);

    int obs_x, obs_y;
    costmap->worldToMapNoBounds(cx, cy, obs_x, obs_y);

    for (int i = min_i; i < max_i; i++)
        for (int j = min_j; j < max_j; j++)
            if (((obs_x - i)*(obs_x - i) + (obs_y - j)*(obs_y - j)) < radius*radius) 
                costmap->setCost(i, j, cost_);

        
}

void SimpleLayer::cbPoint(const geometry_msgs::PointStamped& point){

    boost::recursive_mutex::scoped_lock lock(lock_);
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

void SimpleLayer::cbGPoint(const geometry_msgs::PointStamped& point){

    // boost::recursive_mutex::scoped_lock lock(lock_);
    // ROS_INFO("Got new point");

    // std::string global_frame = layered_costmap_->getGlobalFrameID();
    // geometry_msgs::PointStamped in, out;

    // if (point.header.frame_id != global_frame){
    //     in.header.frame_id = point.header.frame_id;
    //     in.header.stamp = point.header.stamp;
    //     in.point = point.point;

    //     tf_->transform(in, out, global_frame);

    //     transformedPoints_.push_back(out);        
    //     ROS_INFO("Transformed point from %s frame to %s frame", point.header.frame_id.c_str(), global_frame.c_str());
    // }
    // else{
    //     transformedPoints_.push_back(point);
    // }

}
void SimpleLayer::reconfigure(SimpleLayerConfig& config, uint32_t level){
    enabled_ = config.enabled;
    size_ = config.size;
    cost_ = static_cast<unsigned char>(config.cost);
}

}