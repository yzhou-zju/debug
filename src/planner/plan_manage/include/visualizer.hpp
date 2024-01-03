#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher wayPointsPub;
    ros::Publisher routePub;
    ros::Publisher swarmPub;
    ros::Publisher esdf_pub;
    ros::Publisher _all_map_pub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/agent", 10);
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        swarmPub = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm", 10);
        esdf_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map", 1);
        _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
    }

    // Visualize the trajectory and its front-end path
    inline void get_esdf_vis(sensor_msgs::PointCloud2 esdf_get, pcl::PointCloud<pcl::PointXYZ> point_obs)
    {
        sensor_msgs::PointCloud2 globalMap_pcd;
        pcl::toROSMsg(point_obs, globalMap_pcd);
        globalMap_pcd.header.frame_id = "map";
        _all_map_pub.publish(globalMap_pcd);
        esdf_pub.publish(esdf_get);
    }
    inline void visualize(const std::vector<std::vector<Eigen::Vector3d>> &route, int trajId)
    {
        visualization_msgs::Marker routeMarker;
        routeMarker.id = trajId;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.15;

        if (route[trajId].size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route[trajId])
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub.publish(routeMarker);
        }
    }

    inline void swarm_visualize(const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker;

        routeMarker.id = 999;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 0.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 1.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.05;

        if (route.size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            swarmPub.publish(routeMarker);
        }

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 0.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 1.00;
        wayPointsMarker.scale.x = 0.40;
        wayPointsMarker.scale.y = 0.40;
        wayPointsMarker.scale.z = 0.40;
        if (route.size() > 0)
        {
            // Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < route.size(); i++)
            {
                geometry_msgs::Point point;
                point.x = route[i](0);
                point.y = route[i](1);
                point.z = route[i](2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }
    }
};

#endif