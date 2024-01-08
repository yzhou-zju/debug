#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <random>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define IN_CLOSE 97
#define IN_OPEN 98
#define IS_UNKNOWN 99

using namespace std;

namespace trailer_planner
{

    class GridMap
    {
    public:
        // params
        long buffer_size;
        double resolution, resolution_inv;
        Eigen::Vector3d map_origin;
        Eigen::Vector3d map_size;
        Eigen::Vector3d min_boundary;
        Eigen::Vector3d max_boundary;
        Eigen::Vector3i min_idx;
        Eigen::Vector3i max_idx;
        Eigen::Vector3i voxel_num;

    private:
        // datas
        // GridNodePtr *grid_node_map = nullptr;
        vector<char> occ_buffer;
        vector<double> esdf_buffer;
        sensor_msgs::PointCloud2 esdf_cloud;
        pcl::PointCloud<pcl::PointXYZ> point_obs;

        // ros
        bool map_ready = false;
        ros::Timer vis_timer;
        ros::Publisher esdf_pub;
        ros::Subscriber cloud_sub;

    public:
        GridMap() {}
        ~GridMap()
        {
            // for (int i = 0; i < buffer_size; i++)
            //     delete[] grid_node_map[i];

            // delete[] grid_node_map;
            // return;
        }

        void init();
        void updateESDF2d();
        void visCallback(const ros::TimerEvent & /*event*/);
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        // void cloudCallback(const sensor_msgs::PointCloud2 msg);
        void cloudCallback();
        void Generate_cloud(pcl::PointCloud<pcl::PointXYZ> &pc_);
        void get_esdf_map(sensor_msgs::PointCloud2 &esdf_cloud_);
        void get_obs(pcl::PointCloud<pcl::PointXYZ> &point_obs_);
        // std::pair<std::vector<Eigen::Vector2d>, double> astarPlan(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

        double getResolution();
        Eigen::Vector3d getOrigin();
        Eigen::Vector3i get_mapsize();

        inline void getDistance(const Eigen::Vector3d &pos, double &distance);
        inline void getDistance_i(const Eigen::Vector3i &pos, double &distance);
        inline void getDisWithGradI(const Eigen::Vector3d &pos, double &distance, Eigen::Vector3d &grad);
        inline bool isCollision(const Eigen::Vector3d &pos);
        inline void boundIndex(Eigen::Vector3i &id);
        inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
        inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
        inline int toAddress(const Eigen::Vector3i &id);
        inline int toAddress(const int &x, const int &y, const int &z);
        inline bool isInMap(const Eigen::Vector3d &pos);
        inline bool isInMap(const Eigen::Vector3i &idx);
        inline bool isOccupancy(const Eigen::Vector3d &pos);
        inline bool isOccupancy(const Eigen::Vector3i &id);
        inline bool isLineOccupancy(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
        inline int getGridNum();
        inline bool mapReady();

        typedef shared_ptr<GridMap> Ptr;
        typedef unique_ptr<GridMap> UniPtr;
    };

    inline void GridMap::getDistance_i(const Eigen::Vector3i &pos, double &distance)
    {
        Eigen::Vector3d pos_d;
        indexToPos(pos,pos_d);
        getDistance(pos_d, distance);
    }
    inline void GridMap::getDistance(const Eigen::Vector3d &pos, double &distance)
    {
        if (!isInMap(pos))
        {
            distance = 1e+10;
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;
        pos_m(2) -= 0.5 * resolution;

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;
        diff(2) *= resolution_inv;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    boundIndex(current_idx);
                    values[x][y][z] = esdf_buffer[toAddress(current_idx)];
                }
            }

        // value & grad
        // double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        // double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        // double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        // distance = v0 * (1 - diff[1]) + v1 * diff[1];

        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;                          // e
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;                          // f

        distance = (1 - diff(2)) * v0 + diff(2) * v1;

        return;
    }

    inline void GridMap::getDisWithGradI(const Eigen::Vector3d &pos_, double &distance, Eigen::Vector3d &grad_)
    {
        Eigen::Vector3d pos = pos_;
        Eigen::Vector3d grad;
        if (!isInMap(pos))
        {
            distance = 0.0;
            grad.setZero();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;
        pos_m(2) -= 0.5 * resolution;

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;
        diff(2) *= resolution_inv;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    boundIndex(current_idx);
                    values[x][y][z] = esdf_buffer[toAddress(current_idx)];
                }
            }

        // trilinear interpolation
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;                          // e
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;                          // f

        distance = (1 - diff(2)) * v0 + diff(2) * v1;

        grad[2] = (v1 - v0) * resolution_inv;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= resolution_inv;
        grad_ = grad;
        // // value & grad
        // double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        // double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        // distance = v0 * (1 - diff[1]) + v1 * diff[1];
        // grad(1) = (v1 - v0) * resolution_inv;
        // grad(0) = (1 - diff[1]) * (values[1][1] - values[0][1]);
        // grad(0) += diff[1] * (values[1][0] - values[0][0]);
        // grad(0) *= resolution_inv;
        // grad_ = {grad(0), grad(1), 0};
        return;
    }

    inline bool GridMap::isCollision(const Eigen::Vector3d &pos)
    {
        if (isInMap(pos))
        {
            double dist;
            getDistance(pos, dist);
            return dist > 0.0;
        }
        else
            return true;
    }

    inline void GridMap::boundIndex(Eigen::Vector3i &id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));
        id(2) = max(min(id(2), max_idx(2)), min_idx(2));

        return;
    }

    inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
    {
        id(0) = floor((pos(0) - map_origin(0)) * resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * resolution_inv);
        id(2) = floor((pos(2) - map_origin(2)) * resolution_inv);
        return;
    }

    inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
    {
        pos(0) = (id(0) + 0.5) * resolution + map_origin(0);
        pos(1) = (id(1) + 0.5) * resolution + map_origin(1);
        pos(2) = (id(2) + 0.5) * resolution + map_origin(2);
        return;
    }

    inline int GridMap::toAddress(const Eigen::Vector3i &id)
    {
        return id(0) * voxel_num(1) * voxel_num(2) + id(1) * voxel_num(2) + id(2);
    }

    inline int GridMap::toAddress(const int &x, const int &y, const int &z)
    {
        return x * voxel_num(1) * voxel_num(2) + y * voxel_num(2) + z;
    }

    inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
    {
        if (pos(0) < min_boundary(0) + 1e-4 ||
            pos(1) < min_boundary(1) + 1e-4 ||
            pos(2) < min_boundary(2) + 1e-4)
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 ||
            pos(1) > max_boundary(1) - 1e-4 ||
            pos(1) > max_boundary(2) - 1e-4)
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 ||
            idx(1) > voxel_num(1) - 1 ||
            idx(2) > voxel_num(2) - 1)
        {
            // std::cout<<"idx::"<<idx<<std::endl;
            // std::cout<<"voxel_num::"<<voxel_num<<std::endl;
            return false;
        }

        return true;
    }

    inline bool GridMap::isOccupancy(const Eigen::Vector3d &pos)
    {
        Eigen::Vector3i id;

        posToIndex(pos, id);
        return isOccupancy(id);
    }

    inline bool GridMap::isOccupancy(const Eigen::Vector3i &id)
    {
        if (!isInMap(id))
        {
            std::cout<<"out map@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<std::endl;
            return true;
        }

        return occ_buffer[toAddress(id)] == 1;
    }

    inline bool GridMap::isLineOccupancy(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
        bool occ = false;
        Eigen::Vector3d diff = p2 - p1;
        int max_diff = (diff * resolution_inv).lpNorm<Eigen::Infinity>() / 0.2;
        Eigen::Vector3d step = diff / (1.0 * max_diff);

        for (int i = 1; i < max_diff; i++)
        {
            Eigen::Vector3d pt = p1 + step * i;
            if (isOccupancy(pt))
            {
                occ = true;
                break;
            }
        }

        return occ;
    }

    inline int GridMap::getGridNum()
    {
        return voxel_num(0) * voxel_num(1);
    }

    inline bool GridMap::mapReady()
    {
        return map_ready;
    }
}