#include "optimizer/grid_map.h"
namespace trailer_planner
{
    void GridMap::init()
    {
        // nh.getParam("grid_map/map_size_x", map_size[0]);
        // nh.getParam("grid_map/map_size_y", map_size[1]);
        // nh.getParam("grid_map/resolution", resolution);
        resolution = 0.1;
        map_size[0] = 40;
        map_size[1] = 40;
        // esdf_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map", 1);
        // cloud_sub = nh.subscribe("/global_map", 1, &GridMap::cloudCallback, this);
        // vis_timer = nh.createTimer(ros::Duration(1.0), &GridMap::visCallback, this);

        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        map_origin = min_boundary;
        // map_origin = {0, 0};

        // resolution
        resolution_inv = 1.0 / resolution;

        // voxel num
        voxel_num(0) = ceil(map_size(0) / resolution);
        voxel_num(1) = ceil(map_size(1) / resolution);

        // idx
        min_idx = Eigen::Vector2i::Zero();
        max_idx = voxel_num - Eigen::Vector2i::Ones();

        // datas
        buffer_size = voxel_num(0) * voxel_num(1);
        esdf_buffer = vector<double>(buffer_size, 0.0);
        occ_buffer = vector<char>(buffer_size, 0);
        grid_node_map = new GridNodePtr[buffer_size];
        for (int i = 0; i < buffer_size; i++)
            grid_node_map[i] = new GridNode();
        map_ready = false;
        cloudCallback();
        return;
    }

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size)
    {
        int v[size];
        double z[size + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++)
        {
            k++;
            double s;

            do
            {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++)
        {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void GridMap::updateESDF2d()
    {
        int rows = voxel_num[0];
        int cols = voxel_num[1];

        Eigen::MatrixXd tmp_buffer;
        Eigen::MatrixXd neg_buffer;
        Eigen::MatrixXi neg_map;
        Eigen::MatrixXd dist_buffer;
        tmp_buffer.resize(rows, cols);
        neg_buffer.resize(rows, cols);
        neg_map.resize(rows, cols);
        dist_buffer.resize(rows, cols);

        /* ========== compute positive DT ========== */

        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return occ_buffer[toAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max();
                },
                [&](int y, double val)
                { tmp_buffer(x, y) = val; },
                min_idx[1],
                max_idx[1], cols);
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x)
                { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows);
        }

        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (occ_buffer[toAddress(x, y)] == 0)
                {
                    neg_map(x, y) = 1;
                }
                else if (occ_buffer[toAddress(x, y)] == 1)
                {
                    neg_map(x, y) = 0;
                }
                else
                {
                    ROS_ERROR("what?");
                }
            }

        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ? 0 : std::numeric_limits<double>::max();
                },
                [&](int y, double val)
                { tmp_buffer(x, y) = val; },
                min_idx[1],
                max_idx[1], cols);
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x)
                { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows);
        }

        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer[toAddress(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer[toAddress(x, y)] += (-neg_buffer(x, y) + resolution);
            }

        return;
    }
    void GridMap::Generate_cloud(pcl::PointCloud<pcl::PointXYZ> &pc_)
    {
        pcl::PointXYZ pt;
        double cub_long = 5;
        pcl::PointXYZ pt_random;
        // std::random_device rd;
        // default_random_engine eng(rd());
        default_random_engine eng;
        vector<Eigen::Vector2d> obs_position;
        uniform_real_distribution<double> rand_x;
        uniform_real_distribution<double> rand_y;
        uniform_real_distribution<double> rand_w;
        uniform_real_distribution<double> rand_h;
        uniform_real_distribution<double> rand_inf;
        uniform_real_distribution<double> rand_radius_;
        uniform_real_distribution<double> rand_radius2_;
        uniform_real_distribution<double> rand_theta_;
        uniform_real_distribution<double> rand_z_;
        double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
        _x_l = min_boundary[0];
        _y_l = min_boundary[1];
        _x_h = max_boundary[0];
        _y_h = max_boundary[1];
        _w_l = 0.3;
        _w_h = 0.8;
        _h_l = 3;
        _h_h = 7;
        double radius_l_ = 0.05;
        double radius_h_ = 0.08;
        double theta_ = 0.04;
        double z_l_ = 0.0;
        double z_h_ = 3.8;
        int _obs_num = 300;
        double _min_dist = 0.8;
        double _resolution = resolution;
        rand_x = uniform_real_distribution<double>(_x_l, _x_h);
        rand_y = uniform_real_distribution<double>(_y_l, _y_h);
        rand_w = uniform_real_distribution<double>(_w_l, _w_h);
        rand_h = uniform_real_distribution<double>(_h_l, _h_h);
        rand_inf = uniform_real_distribution<double>(0.5, 1.5);

        rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
        rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
        rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
        rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

        // generate polar obs
        for (int i = 0; i < _obs_num && ros::ok(); i++)
        {
            double x, y, w, h, inf;
            x = rand_x(eng);
            y = rand_y(eng);
            w = rand_w(eng);
            inf = rand_inf(eng);

            bool flag_continue = false;
            for (auto p : obs_position) // 如果距离太近，新生成的障碍物不能用，重新生成
                if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
                {
                    i--;
                    flag_continue = true;
                    break;
                }
            if (flag_continue)
                continue;

            obs_position.push_back(Eigen::Vector2d(x, y));

            x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            y = floor(y / _resolution) * _resolution + _resolution / 2.0;

            int widNum = ceil((w * inf) / _resolution);
            double radius = (w * inf) / 2;

            for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
                for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
                {
                    h = rand_h(eng);
                    int heiNum = ceil(h / _resolution);
                    for (int t = -10; t < heiNum; t++)
                    {
                        double temp_x = x + (r + 0.5) * _resolution + 1e-2;
                        double temp_y = y + (s + 0.5) * _resolution + 1e-2;
                        double temp_z = (t + 0.5) * _resolution + 1e-2;
                        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
                        {
                            pt_random.x = temp_x;
                            pt_random.y = temp_y;
                            pt_random.z = temp_z;
                            pc_.points.push_back(pt_random);
                        }
                    }
                }
        }
        point_obs = pc_;
    }
    void GridMap::cloudCallback()
    {
        if (map_ready)
            return;

        pcl::PointCloud<pcl::PointXYZ> pc;
        Generate_cloud(pc);
        // pcl::fromROSMsg(msg, pc);

        for (size_t i = 0; i < pc.points.size(); i++)
        {
            Eigen::Vector2i id;
            posToIndex(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap(id))
                occ_buffer[toAddress(id)] = 1;
        }

        updateESDF2d();
        pc.clear();
        // test esdf computation
        for (int i = 0; i < voxel_num(0); i++)
        {
            for (int j = 0; j < voxel_num(1); j++)
            {
                Eigen::Vector2i id(i, j);
                Eigen::Vector2d pos;
                indexToPos(id, pos);
                pcl::PointXYZ p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = esdf_buffer[toAddress(id)];
                pc.push_back(p);
            }
        }

        pc.header.frame_id = "map";
        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;
        pcl::toROSMsg(pc, esdf_cloud);

        map_ready = true;

        return;
    }
    void GridMap::get_esdf_map(sensor_msgs::PointCloud2 &esdf_cloud_)
    {
        esdf_cloud_ = esdf_cloud;
    }
    void GridMap::get_obs(pcl::PointCloud<pcl::PointXYZ> &point_obs_)
    {
        point_obs_ = point_obs;
    }
    // void GridMap::visCallback(const ros::TimerEvent & /*event*/)
    // {
    //     if (!map_ready)
    //         return;
    //     esdf_pub.publish(esdf_cloud);
    //     return;
    // }

}