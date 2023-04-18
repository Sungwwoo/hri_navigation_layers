#ifndef _VIRTUAL_NAVIGATION_LAYER_H_
#define _VIRTUAL_NAVIGATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/inflation_layer.h>
#include <geometry_msgs/PointStamped.h>
#include <virtual_navigation_layers/VirtualLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>
#include <list>

namespace virtual_navigation_layers{
class VirtualLayer : public costmap_2d::Layer{
public:
    VirtualLayer();

    /**
     * @brief Called at the end of costmap_2d::Layer initialization
     * 
     */
    virtual void onInitialize();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:

    void cbPoint(const geometry_msgs::PointStamped& point);
    void cbGPoint(const geometry_msgs::PointStamped& point);
    void reconfigure(VirtualLayerConfig& config, uint32_t level);


    ros::Subscriber sub_point_, g_sub_point_;
    ros::Publisher pub_clicked_point_marker_;
    geometry_msgs::PointStamped point_;
    std::list<geometry_msgs::PointStamped> transformedPoints_;
    visualization_msgs::MarkerArray removeMarkers_;
    boost::recursive_mutex lock_;
    bool first_time_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_, size_;
    unsigned char cost_;
    
    // Dynamic reconfigure objects
    dynamic_reconfigure::Server<VirtualLayerConfig>* dsrv_;
    dynamic_reconfigure::Server<VirtualLayerConfig>::CallbackType cb_;
// private:
//     costmap_2d::InflationLayer* inflation_layer_;
};
}


#endif