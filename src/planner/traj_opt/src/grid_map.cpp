#include "optimizer/grid_map.h"
namespace trailer_planner
{
    void GridMap::init()
    {
        // nh.getParam("grid_map/map_size_x", map_size[0]);
        // nh.getParam("grid_map/map_size_y", map_size[1]);
        // nh.getParam("grid_map/resolution", resolution);
        std::cout << "here   map3;;;;" << std::endl;
        resolution = 0.2;
        map_size[0] = 80;
        map_size[1] = 28;
        map_size[2] = 9;
        // esdf_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map", 1);
        // cloud_sub = nh.subscribe("/global_map", 1, &GridMap::cloudCallback, this);
        // vis_timer = nh.createTimer(ros::Duration(1.0), &GridMap::visCallback, this);

        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        map_origin = min_boundary;
        map_origin[2] = 0;
        // map_origin = {0, -11, 0};

        // resolution
        resolution_inv = 1.0 / resolution;
        std::cout << "here   map4;;;;" << std::endl;
        // voxel num
        voxel_num(0) = ceil(map_size(0) / resolution);
        voxel_num(1) = ceil(map_size(1) / resolution);
        voxel_num(2) = ceil(map_size(2) / resolution);

        // idx
        min_idx = Eigen::Vector3i::Zero();
        max_idx = voxel_num - Eigen::Vector3i::Ones();
        // datas
        buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
        esdf_buffer = vector<double>(buffer_size, 0.0);
        occ_buffer = vector<char>(buffer_size, 0);
        // grid_node_map = new GridNodePtr[buffer_size];
        // for (int i = 0; i < buffer_size; i++)
        //     grid_node_map[i] = new GridNode();
        map_ready = false;
        cloudCallback();
        return;
    }
    double GridMap::getResolution()
    {
        return resolution;
    }
    Eigen::Vector3d GridMap::getOrigin()
    {
        return map_origin;
    }
    Eigen::Vector3i GridMap::get_mapsize()
    {
        return voxel_num;
    }

    template <typename F_get_val, typename F_set_val>
    void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size)
    {
        int v[voxel_num(size)];
        double z[voxel_num(size) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();
        for (int q = start + 1; q <= end; q++)
        {
            k++;
            double s;
            // std::cout << "qqqqqqqqq:" << q << std::endl;
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
        int x_s = voxel_num[0];
        int y_s = voxel_num[1];
        int z_s = voxel_num[2];
        std::vector<double> tmp_buffer1;
        std::vector<double> tmp_buffer2;
        std::vector<char> neg_buffer;
        std::vector<char> neg_map;
        std::vector<double> dist_buffer;
        tmp_buffer1 = std::vector<double>(x_s * y_s * z_s, 0);
        tmp_buffer2 = std::vector<double>(x_s * y_s * z_s, 0);
        neg_buffer = std::vector<char>(x_s * y_s * z_s, 0);
        neg_map = std::vector<char>(x_s * y_s * z_s, 0);
        dist_buffer = std::vector<double>(x_s * y_s * z_s, 10000);
        std::cout << "here33311113" << std::endl;
        /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            for (int y = min_idx[1]; y <= max_idx[1]; y++)
            {
                fillESDF(
                    [&](int z)
                    {
                        return occ_buffer[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();
                    },
                    [&](int z, double val)
                    { tmp_buffer1[toAddress(x, y, z)] = val; },
                    min_idx[2],
                    max_idx[2], 2);
            }
        }
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            for (int z = min_idx[2]; z <= max_idx[2]; z++)
            {
                fillESDF([&](int y)
                         { return tmp_buffer1[toAddress(x, y, z)]; },
                         [&](int y, double val)
                         { tmp_buffer2[toAddress(x, y, z)] = val; },
                         min_idx[1],
                         max_idx[1], 1);
            }
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            for (int z = min_idx[2]; z <= max_idx[2]; z++)
            {
                fillESDF([&](int x)
                         { return tmp_buffer2[toAddress(x, y, z)]; },
                         [&](int x, double val)
                         {
                             dist_buffer[toAddress(x, y, z)] = resolution * std::sqrt(val);
                             //  min(mp_.resolution_ * std::sqrt(val),
                             //      distance_buffer_[toAddress(x, y, z)]);
                         },
                         min_idx[0], max_idx[0], 0);
            }
        }

        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
                for (int z = min_idx(2); z <= max_idx(2); ++z)
                {

                    int idx = toAddress(x, y, z);
                    if (occ_buffer[idx] == 0)
                    {
                        neg_map[idx] = 1;
                    }
                    else if (occ_buffer[idx] == 1)
                    {
                        neg_map[idx] = 0;
                    }
                    else
                    {
                        ROS_ERROR("what?");
                    }
                }

        tmp_buffer1.clear();
        tmp_buffer2.clear();

        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            for (int y = min_idx[1]; y <= max_idx[1]; y++)
            {
                fillESDF(
                    [&](int z)
                    {
                        return neg_map[x * voxel_num(1) * voxel_num(2) +
                                       y * voxel_num(2) + z] == 1
                                   ? 0
                                   : std::numeric_limits<double>::max();
                    },
                    [&](int z, double val)
                    { tmp_buffer1[toAddress(x, y, z)] = val; },
                    min_idx[2],
                    max_idx[2], 2);
            }
        }

        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            for (int z = min_idx[2]; z <= max_idx[2]; z++)
            {
                fillESDF([&](int y)
                         { return tmp_buffer1[toAddress(x, y, z)]; },
                         [&](int y, double val)
                         { tmp_buffer2[toAddress(x, y, z)] = val; },
                         min_idx[1],
                         max_idx[1], 1);
            }
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            for (int z = min_idx[2]; z <= max_idx[2]; z++)
            {
                fillESDF([&](int x)
                         { return tmp_buffer2[toAddress(x, y, z)]; },
                         [&](int x, double val)
                         {
                             neg_buffer[toAddress(x, y, z)] = resolution * std::sqrt(val);
                         },
                         min_idx[0], max_idx[0], 0);
            }
        }

        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
                for (int z = min_idx(2); z <= max_idx(2); ++z)
                {

                    int idx = toAddress(x, y, z);
                    esdf_buffer[idx] = dist_buffer[idx];

                    if (neg_buffer[idx] > 0.0)
                        esdf_buffer[idx] += (-neg_buffer[idx] + resolution);
                }

        std::cout << "here3333333" << std::endl;
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
        _y_h = max_boundary[1]-10;
        std::cout<<"x_l:"<<_x_l<<std::endl;
        std::cout<<"y_l:"<<_y_l<<std::endl;
        std::cout<<"x_h:"<<_x_h<<std::endl;
        std::cout<<"y_h:"<<_y_h<<std::endl;
        // _x_l = 0;
        // _y_l = 0;
        // _x_h = map_size[0];
        // _y_h = map_size[1];
        _w_l = 0.3;
        _w_h = 0.8;
        _h_l = 3;
        _h_h = 7;
        double radius_l_ = 0.05;
        double radius_h_ = 0.08;
        double theta_ = 0.04;
        double z_l_ = 0.0;
        double z_h_ = 5.8;
        int _obs_num = 150;
        int circle_num_ = 50;
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

        // generate circle obs
        for (int i = 0; i < circle_num_; ++i)
        {
            double x, y, z;
            x = rand_x(eng);
            y = rand_y(eng);
            z = rand_z_(eng);

            x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            y = floor(y / _resolution) * _resolution + _resolution / 2.0;
            z = floor(z / _resolution) * _resolution + _resolution / 2.0;

            Eigen::Vector3d translate(x, y, z);

            double theta = rand_theta_(eng);
            Eigen::Matrix3d rotate;
            rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
                1;

            // double radius1 = rand_radius_(eng);
            // double radius2 = rand_radius2_(eng);
            double radius1 = 1;
            double radius2 = 1;

            // draw a circle centered at (x,y,z)
            Eigen::Vector3d cpt;
            for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
            {
                cpt(0) = 0.0;
                cpt(1) = radius1 * cos(angle);
                cpt(2) = radius2 * sin(angle);

                // inflate
                Eigen::Vector3d cpt_if;
                for (int ifx = -0; ifx <= 0; ++ifx)
                    for (int ify = -0; ify <= 0; ++ify)
                        for (int ifz = -0; ifz <= 0; ++ifz)
                        {
                            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                           ifz * _resolution);
                            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                            pt_random.x = cpt_if(0);
                            pt_random.y = cpt_if(1);
                            pt_random.z = cpt_if(2);
                            // cloudMap.push_back(pt_random);
                            pc_.points.push_back(pt_random);
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

        for (long i = 0; i < pc.points.size(); i++)
        {
            Eigen::Vector3i id;
            posToIndex(Eigen::Vector3d(pc.points[i].x, pc.points[i].y, pc.points[i].z), id);
            if (isInMap(id))
                occ_buffer[toAddress(id)] = 1;
        }

        updateESDF2d();
        pc.clear();
        pcl::PointCloud<pcl::PointXYZI> pc_i;
        // test esdf computation
        double dist_int;
        double min_dist = 0;
        double max_dist = 3;
        for (int i = 0; i < voxel_num(0); i++)
        {
            for (int j = 0; j < voxel_num(1)-48; j++)
            {
                for (int z = 0; z < voxel_num(2); z++)
                {
                    Eigen::Vector3i id(i, j, z);
                    Eigen::Vector3d pos;
                    indexToPos(Eigen::Vector3i(i, j, -0.5), pos);
                    pos(2) = 0;
                    pcl::PointXYZI p;
                    p.x = pos(0);
                    p.y = pos(1);
                    // p.z = esdf_buffer[toAddress(id)];
                    p.z = pos(2);
                    getDistance(pos, dist_int);
                    dist_int = min(dist_int, max_dist);
                    dist_int = max(dist_int, min_dist);
                    p.intensity = (dist_int - min_dist) / (max_dist - min_dist);
                    pc_i.push_back(p);
                }
            }
        }

        pc_i.header.frame_id = "map";
        pc_i.width = pc_i.points.size();
        pc_i.height = 1;
        pc_i.is_dense = true;
        pcl::toROSMsg(pc_i, esdf_cloud);

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