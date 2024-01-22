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
#include <random>
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
    std::vector<ros::Publisher> routePub;
    std::vector<ros::Publisher> swarmPub;
    ros::Publisher esdf_pub;
    ros::Publisher _all_map_pub;
    ros::Publisher pathPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        swarmPub.resize(70);
        routePub.resize(70);
        pathPub = nh.advertise<visualization_msgs::Marker>("/visualizer/astar_path", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/agent", 10);
        routePub[0] = nh.advertise<visualization_msgs::Marker>("/visualizer/route0", 10);
        routePub[1] = nh.advertise<visualization_msgs::Marker>("/visualizer/route1", 10);
        routePub[2] = nh.advertise<visualization_msgs::Marker>("/visualizer/route2", 10);
        routePub[3] = nh.advertise<visualization_msgs::Marker>("/visualizer/route3", 10);
        routePub[4] = nh.advertise<visualization_msgs::Marker>("/visualizer/route4", 10);
        routePub[5] = nh.advertise<visualization_msgs::Marker>("/visualizer/route5", 10);
        routePub[6] = nh.advertise<visualization_msgs::Marker>("/visualizer/route6", 10);
        routePub[7] = nh.advertise<visualization_msgs::Marker>("/visualizer/route7", 10);
        routePub[8] = nh.advertise<visualization_msgs::Marker>("/visualizer/route8", 10);
        routePub[9] = nh.advertise<visualization_msgs::Marker>("/visualizer/route9", 10);
        routePub[10] = nh.advertise<visualization_msgs::Marker>("/visualizer/route10", 10);
        routePub[11] = nh.advertise<visualization_msgs::Marker>("/visualizer/route11", 10);
        routePub[12] = nh.advertise<visualization_msgs::Marker>("/visualizer/route12", 10);
        routePub[13] = nh.advertise<visualization_msgs::Marker>("/visualizer/route13", 10);
        routePub[14] = nh.advertise<visualization_msgs::Marker>("/visualizer/route14", 10);
        routePub[15] = nh.advertise<visualization_msgs::Marker>("/visualizer/route15", 10);
        routePub[16] = nh.advertise<visualization_msgs::Marker>("/visualizer/route16", 10);
        routePub[17] = nh.advertise<visualization_msgs::Marker>("/visualizer/route17", 10);
        routePub[18] = nh.advertise<visualization_msgs::Marker>("/visualizer/route18", 10);
        routePub[19] = nh.advertise<visualization_msgs::Marker>("/visualizer/route19", 10);
        routePub[20] = nh.advertise<visualization_msgs::Marker>("/visualizer/route20", 10);
        routePub[21] = nh.advertise<visualization_msgs::Marker>("/visualizer/route21", 10);
        routePub[22] = nh.advertise<visualization_msgs::Marker>("/visualizer/route22", 10);
        routePub[23] = nh.advertise<visualization_msgs::Marker>("/visualizer/route23", 10);
        routePub[24] = nh.advertise<visualization_msgs::Marker>("/visualizer/route24", 10);
        routePub[25] = nh.advertise<visualization_msgs::Marker>("/visualizer/route25", 10);
        routePub[26] = nh.advertise<visualization_msgs::Marker>("/visualizer/route26", 10);
        routePub[27] = nh.advertise<visualization_msgs::Marker>("/visualizer/route27", 10);
        routePub[28] = nh.advertise<visualization_msgs::Marker>("/visualizer/route28", 10);
        routePub[29] = nh.advertise<visualization_msgs::Marker>("/visualizer/route29", 10);
        routePub[30] = nh.advertise<visualization_msgs::Marker>("/visualizer/route30", 10);

        routePub[31] = nh.advertise<visualization_msgs::Marker>("/visualizer/route31", 10);
        routePub[32] = nh.advertise<visualization_msgs::Marker>("/visualizer/route32", 10);
        routePub[33] = nh.advertise<visualization_msgs::Marker>("/visualizer/route33", 10);
        routePub[34] = nh.advertise<visualization_msgs::Marker>("/visualizer/route34", 10);
        routePub[35] = nh.advertise<visualization_msgs::Marker>("/visualizer/route35", 10);
        routePub[36] = nh.advertise<visualization_msgs::Marker>("/visualizer/route36", 10);
        routePub[37] = nh.advertise<visualization_msgs::Marker>("/visualizer/route37", 10);
        routePub[38] = nh.advertise<visualization_msgs::Marker>("/visualizer/route38", 10);
        routePub[39] = nh.advertise<visualization_msgs::Marker>("/visualizer/route39", 10);
        routePub[40] = nh.advertise<visualization_msgs::Marker>("/visualizer/route40", 10);
        routePub[41] = nh.advertise<visualization_msgs::Marker>("/visualizer/route41", 10);
        routePub[42] = nh.advertise<visualization_msgs::Marker>("/visualizer/route42", 10);
        routePub[43] = nh.advertise<visualization_msgs::Marker>("/visualizer/route43", 10);
        routePub[44] = nh.advertise<visualization_msgs::Marker>("/visualizer/route44", 10);
        routePub[45] = nh.advertise<visualization_msgs::Marker>("/visualizer/route45", 10);
        routePub[46] = nh.advertise<visualization_msgs::Marker>("/visualizer/route46", 10);
        routePub[47] = nh.advertise<visualization_msgs::Marker>("/visualizer/route47", 10);
        routePub[48] = nh.advertise<visualization_msgs::Marker>("/visualizer/route48", 10);
        routePub[49] = nh.advertise<visualization_msgs::Marker>("/visualizer/route49", 10);
        routePub[50] = nh.advertise<visualization_msgs::Marker>("/visualizer/route50", 10);
        routePub[51] = nh.advertise<visualization_msgs::Marker>("/visualizer/route51", 10);
        routePub[52] = nh.advertise<visualization_msgs::Marker>("/visualizer/route52", 10);
        routePub[53] = nh.advertise<visualization_msgs::Marker>("/visualizer/route53", 10);
        routePub[54] = nh.advertise<visualization_msgs::Marker>("/visualizer/route54", 10);
        routePub[55] = nh.advertise<visualization_msgs::Marker>("/visualizer/route55", 10);
        routePub[56] = nh.advertise<visualization_msgs::Marker>("/visualizer/route56", 10);
        routePub[57] = nh.advertise<visualization_msgs::Marker>("/visualizer/route57", 10);
        routePub[58] = nh.advertise<visualization_msgs::Marker>("/visualizer/route58", 10);
        routePub[59] = nh.advertise<visualization_msgs::Marker>("/visualizer/route59", 10);
        routePub[60] = nh.advertise<visualization_msgs::Marker>("/visualizer/route60", 10);
        routePub[61] = nh.advertise<visualization_msgs::Marker>("/visualizer/route61", 10);
        routePub[62] = nh.advertise<visualization_msgs::Marker>("/visualizer/route62", 10);
        routePub[63] = nh.advertise<visualization_msgs::Marker>("/visualizer/route63", 10);
        routePub[64] = nh.advertise<visualization_msgs::Marker>("/visualizer/route64", 10);
        // routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        swarmPub[0] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm0", 10);
        swarmPub[1] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm1", 10);
        swarmPub[2] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm2", 10);
        swarmPub[3] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm3", 10);
        swarmPub[4] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm4", 10);
        swarmPub[5] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm5", 10);
        swarmPub[6] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm6", 10);
        swarmPub[7] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm7", 10);
        swarmPub[8] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm8", 10);
        swarmPub[9] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm9", 10);
        swarmPub[10] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm10", 10);
        swarmPub[11] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm11", 10);
        swarmPub[12] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm12", 10);
        swarmPub[13] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm13", 10);
        swarmPub[14] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm14", 10);
        swarmPub[15] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm15", 10);
        swarmPub[16] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm16", 10);
        swarmPub[17] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm17", 10);
        swarmPub[18] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm18", 10);
        swarmPub[19] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm19", 10);
        swarmPub[20] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm20", 10);
        swarmPub[21] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm21", 10);
        swarmPub[22] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm22", 10);
        swarmPub[23] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm23", 10);
        swarmPub[24] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm24", 10);
        swarmPub[25] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm25", 10);
        swarmPub[26] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm26", 10);
        swarmPub[27] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm27", 10);
        swarmPub[28] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm28", 10);
        swarmPub[29] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm29", 10);
        swarmPub[30] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm30", 10);

        swarmPub[31] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm31", 10);
        swarmPub[32] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm32", 10);
        swarmPub[33] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm33", 10);
        swarmPub[34] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm34", 10);
        swarmPub[35] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm35", 10);
        swarmPub[36] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm36", 10);
        swarmPub[37] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm37", 10);
        swarmPub[38] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm38", 10);
        swarmPub[39] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm39", 10);
        swarmPub[40] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm40", 10);
        swarmPub[41] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm41", 10);
        swarmPub[42] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm42", 10);
        swarmPub[43] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm43", 10);
        swarmPub[44] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm44", 10);
        swarmPub[45] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm45", 10);
        swarmPub[46] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm46", 10);
        swarmPub[47] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm47", 10);
        swarmPub[48] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm48", 10);
        swarmPub[49] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm49", 10);
        swarmPub[50] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm50", 10);
        swarmPub[51] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm51", 10);
        swarmPub[52] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm52", 10);
        swarmPub[53] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm53", 10);
        swarmPub[54] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm54", 10);
        swarmPub[55] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm55", 10);
        swarmPub[56] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm56", 10);
        swarmPub[57] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm57", 10);
        swarmPub[58] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm58", 10);
        swarmPub[59] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm59", 10);
        swarmPub[60] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm60", 10);
        swarmPub[61] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm61", 10);
        swarmPub[62] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm62", 10);
        swarmPub[63] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm63", 10);
        swarmPub[64] = nh.advertise<visualization_msgs::Marker>("/visualizer/swarm64", 10);
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

        for (int i = 0; i < route.size(); i++)
        {
            if (route[i].size() > 0)
            {
                bool first = true;
                Eigen::Vector3d last;
                for (auto it : route[i])
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

                routePub[i].publish(routeMarker);
                routeMarker.points.clear();
            }
        }
    }
    inline void astar_path_vis(const std::vector<Eigen::Vector3d> &path)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker;
        routeMarker.id = 999;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;

        routeMarker.color.a = 1.0;
        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 2;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 1.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.30;
        wayPointsMarker.scale.y = 0.30;
        wayPointsMarker.scale.z = 0.30;
        if (path.size() > 0)
        {
            // Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < path.size(); i++)
            {
                geometry_msgs::Point point;
                point.x = path[i](0);
                point.y = path[i](1);
                point.z = path[i](2);
                wayPointsMarker.points.push_back(point);
            }

            pathPub.publish(wayPointsMarker);
        }
    }
    inline void swarm_visualize(const std::vector<std::vector<Eigen::Vector3d>> &route, const std::vector<Eigen::Vector3d> &vall)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker;
        std::default_random_engine generator1;
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        double random_number1;
        double random_number2;
        double random_number3;
        for (int m = 0; m < route.size(); m++)
        {
            random_number1 = distribution(generator1);
            random_number2 = distribution(generator1);
            random_number3 = distribution(generator1);
            routeMarker.id = 999;
            routeMarker.type = visualization_msgs::Marker::LINE_LIST;
            routeMarker.header.stamp = ros::Time::now();
            routeMarker.header.frame_id = "map";
            routeMarker.pose.orientation.w = 1.00;
            routeMarker.action = visualization_msgs::Marker::ADD;
            routeMarker.ns = "route";
            routeMarker.color.r = random_number1;
            routeMarker.color.g = random_number2;
            routeMarker.color.b = random_number3;
            routeMarker.color.a = 0.03;
            routeMarker.scale.x = 0.05;

            if (route[m].size() > 0)
            {
                bool first = true;
                Eigen::Vector3d last;
                for (auto it : route[m])
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

                swarmPub[m].publish(routeMarker);
                routeMarker.points.clear();
            }
        }
        routeMarker.color.a = 1.0;
        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 0.00;
        wayPointsMarker.color.g = 1.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.40;
        wayPointsMarker.scale.y = 0.40;
        wayPointsMarker.scale.z = 0.40;
        if (vall.size() > 0)
        {
            // Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < vall.size(); i++)
            {
                geometry_msgs::Point point;
                point.x = vall[i](0);
                point.y = vall[i](1);
                point.z = vall[i](2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }
    }
};

#endif