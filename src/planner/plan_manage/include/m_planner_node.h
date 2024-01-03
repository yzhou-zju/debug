#ifndef M_PLANNER_NODE_H
#define M_PLANNER_NODE_H
#include "visualizer.hpp"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <optimizer/poly_traj_utils.hpp>
#include <optimizer/poly_traj_optimizer.h>
using namespace ego_planner;
using namespace poly_traj;
using namespace std;

class ego_planner_node
{
public:
  void do_planner(bool m_or_not_get_, const int agent_num_)
  {
    get_txt(agent_num_);
    std::cout << "id_in_group[i] size00:" << id_in_group[0].size() << std::endl;
    id_chose_form = 1;
    group_num_ = 1; // 初始化
    re_setup(agent_num_);
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
    std::vector<int> leader_id_;
    std::vector<Eigen::Vector3d> v_;
    bool m_or_not_get = true;
    bool flag_suss = true;
    traj_.swarm_traj.resize(1);
    have_no_swarm();
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
    bool flag_global;
    // global_traj_opt.reset(new PolyTrajOptimizer);
    // global_traj_opt->setParam(leader_id_, v_);
    // global_traj_opt->setSwarmTrajs(&traj_.swarm_traj);
    // flag_global = global_traj_opt->optimizeTrajectory_lbfgs(headState[0], tailState[0],
    //                                                         intState[0], initT_,
    //                                                         cstr_pts[0], true, 0);
    // traj_global = global_traj_opt->getMinJerkOptPtr()->getTraj();

    for (int i = 0; i < agent_num_; i++)
    {
      formation_optim[i]->setlog(i, a_num_);
      formation_optim[i]->setParam(leader_id_, v_);
      formation_optim[i]->setSwarmTrajs(&traj_.swarm_traj);
      flag_[i] = formation_optim[i]->optimizeTrajectory_lbfgs(headState[i], tailState[i],
                                                              intState[i], initT_,
                                                              cstr_pts[i], true, 0);

      if (!flag_[i])
      {
        flag_suss = false;
      }
      else
      {
        traj_optim[i] = formation_optim[i]->getMinJerkOptPtr()->getTraj();
        traj_global[i] = traj_optim[i];
      }
    }
    group_swarm(m_or_not_get);
    // planner_all_formation_once(0);
    ros::Time t_0 = ros::Time::now();
    planner_all_formation_once(0, t_0, t_0);
    planner_all_formation_once(0, t_0, t_0);
    // planner_all_formation_once(0, t_0, t_0);
    // planner_all_formation_once(0, t_0, t_0);
    ros::Time t_1 = ros::Time::now();
    for (int i = 0; i < agent_num_; i++)
    {
      std::cout << "time total:" << traj_optim[i].getTotalDuration() << std::endl;
      // std::cout << "pos:" << traj_optim[i].getPos(0) << std::endl;
      // std::cout << "pos:" << traj_optim[i].getPos(2.5) << std::endl;
      // std::cout << "pos:" << traj_optim[i].getPos(5) << std::endl;
      // std::cout << "pos:" << traj_optim[i].getPos(7.5) << std::endl;
    }
  }

  void planner_all_formation_once(const double t_planner_go, const ros::Time t_n, const ros::Time t_global_s)
  {
    std::cout << "id_in_group[i] size:" << id_in_group[0].size() << std::endl;
    Eigen::Vector3d pos_begin_;
    Eigen::Vector3d vel_begin_;
    Eigen::Vector3d acc_begin_;
    Eigen::Vector3d pos_end_;
    Eigen::Vector3d vel_end_;
    Eigen::Vector3d acc_end_;
    Eigen::Vector3d int_begin_;
    Eigen::Vector3d int_begin_1;
    Eigen::Vector3d int_begin_2;
    Eigen::VectorXd initT_change;
    initT_change = initT_;
    std::vector<Eigen::Vector3d> v_group; // 小编队的形状
    std::vector<int> id_all;              // 小组成员编号（在大组的编号）
    std::vector<int> num_every_group;     // 每个小组内的数量
    int group_id_;                        // 在哪个小组
    int g_num;                            // 小组数量
    double time_hori = 5;
    num_every_group = get_every_group_num();
    g_num = get_group_num();
    std::cout << "g_num:::" << g_num << std::endl;
    traj_real.resize(g_num);
    for (int i = 0; i < num_every_group.size(); i++)
    {
      traj_real[i].swarm_traj.resize(num_every_group[i]);
    }
    if (t_planner_go != 0)
    {

      for (int i = 0; i < a_num_; i++)
      {
        pos_begin_ = traj_optim[i].getPos(t_planner_go);
        vel_begin_ = traj_optim[i].getVel(t_planner_go);
        acc_begin_ = {0, 0, 0};
        acc_end_ = {0, 0, 0};

        headState[i] << pos_begin_, vel_begin_, acc_begin_;
        // pos_end_2 = pos_end_ + v_all[i];
        if ((t_n - t_global_s).toSec() + time_hori <= traj_global[i].getTotalDuration())
        {
          pos_end_ = traj_global[i].getPos((t_n - t_global_s).toSec() + time_hori);
          vel_end_ = traj_global[i].getVel((t_n - t_global_s).toSec() + time_hori);
        }
        else
        {
          pos_end_ = traj_global[i].getPos(traj_global[i].getTotalDuration());
          vel_end_ = traj_global[i].getVel(traj_global[i].getTotalDuration());
        }
        tailState[i] << pos_end_, vel_end_, acc_end_;

        initT_change[0] = sqrt((pos_end_[0] - pos_begin_[0]) * (pos_end_[0] - pos_begin_[0]) + (pos_end_[1] - pos_begin_[1]) * (pos_end_[1] - pos_begin_[1])) / 4;
        initT_change[1] = sqrt((pos_end_[0] - pos_begin_[0]) * (pos_end_[0] - pos_begin_[0]) + (pos_end_[1] - pos_begin_[1]) * (pos_end_[1] - pos_begin_[1])) / 4;
        initT_change[2] = sqrt((pos_end_[0] - pos_begin_[0]) * (pos_end_[0] - pos_begin_[0]) + (pos_end_[1] - pos_begin_[1]) * (pos_end_[1] - pos_begin_[1])) / 4;
        initT_change[3] = sqrt((pos_end_[0] - pos_begin_[0]) * (pos_end_[0] - pos_begin_[0]) + (pos_end_[1] - pos_begin_[1]) * (pos_end_[1] - pos_begin_[1])) / 4;
        int_begin_ = {(pos_end_[0] - pos_begin_[0]) / 4 + pos_begin_[0], (pos_end_[1] - pos_begin_[1]) / 4 + pos_begin_[1], 1};
        int_begin_1 = {2 * (pos_end_[0] - pos_begin_[0]) / 4 + pos_begin_[0], 2 * (pos_end_[1] - pos_begin_[1]) / 4 + pos_begin_[1], 1};
        int_begin_2 = {3 * (pos_end_[0] - pos_begin_[0]) / 4 + pos_begin_[0], 3 * (pos_end_[1] - pos_begin_[1]) / 4 + pos_begin_[1], 1};
        intState[i] << int_begin_, int_begin_1, int_begin_2;

        std::cout << "pos_begin::" << pos_begin_ << std::endl;
        std::cout << "pos_end::" << pos_end_ << std::endl;
        std::cout << "int_begin_::" << int_begin_ << std::endl;
        std::cout << "int_begin_1::" << int_begin_1 << std::endl;
        std::cout << "int_begin2::" << int_begin_2 << std::endl;
        std::cout << "init ttt::" << initT_change << std::endl;
      }
      initT_ = initT_change;
    }

    std::cout << "all planner!!!" << std::endl;
    for (int j = 0; j < a_num_; j++)
    {
      double t_now_0 = 0;
      int piece_nums = 4;
      int recv_id;
      std::vector<int> leader_id_;
      std::cout << "renew!!!!!!!!!other:" << j << std::endl;
      /*********************************依次更新全部小编队的轨迹*********************************************/
      for (int i = 0; i < g_num; i++)
      {
        id_all = id_in_group[i];
        std::cout<<"iiiiiiiiiiiii:"<<i<<std::endl;
        for(int k=0;k<id_in_group[i].size();k++)
        {
          std::cout<<"id in group:"<<id_in_group[i][k]<<std::endl;
        }
        for (int k = 0; k < num_every_group[i]; k++)
        {
          std::cout << "kkkkkkkkkkkkkkkk:" << k << std::endl;
          traj_real[i].swarm_traj[k].drone_id = k;
          traj_real[i].swarm_traj[k].traj_id = k;
          traj_real[i].swarm_traj[k].start_time = 0;
          traj_real[i].swarm_traj[k].constraint_aware = 0;
          traj_real[i].swarm_traj[k].traj = traj_optim[id_all[k]];
          traj_real[i].swarm_traj[k].duration = traj_optim[id_all[k]].getTotalDuration();
          traj_real[i].swarm_traj[k].start_pos = traj_optim[id_all[k]].getPos(0);
          std::cout << "eenddddddddddddk:" << k << std::endl;
        }
        std::cout << "end iiiiii!!!" << std::endl;
      }
      /*********************************组内轨迹优化*********************************************/
      group_id_ = get_group_and_id(j);
      v_group = get_v_group(group_id_);
      recv_id = get_id_in_group(j);
      std::cout << "recv_id:" << recv_id << std::endl;
      formation_optim[j]->setDroneId(recv_id);
      formation_optim[j]->fisrt_planner_or_not(false);
      formation_optim[j]->setParam(leader_id_, v_group);
      formation_optim[j]->setSwarmTrajs(&traj_real[group_id_].swarm_traj);
      flag_[j] = formation_optim[j]->optimizeTrajectory_lbfgs(headState[j], tailState[j],
                                                              intState[j], initT_change,
                                                              cstr_pts[j], true, t_planner_go);
      std::cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@::"<<j<<std::endl;
      std::cout<<"intState[j]0::"<<headState[j](0,0)<<"   "<<headState[j](1,0)<<"   "<<headState[j](2,0)<<std::endl;
      std::cout<<"intState[j]0::"<<intState[j](0,0)<<"   "<<intState[j](0,1)<<"   "<<intState[j](0,2)<<std::endl;
      std::cout<<"intState[j]1::"<<intState[j](1,0)<<"   "<<intState[j](1,1)<<"   "<<intState[j](1,2)<<std::endl;
      std::cout<<"intState[j]2::"<<intState[j](2,0)<<"   "<<intState[j](2,1)<<"   "<<intState[j](2,2)<<std::endl;
      std::cout<<"intState[j]2::"<<initT_change(0)<<"   "<<initT_change(1)<<"   "<<initT_change(2)<<initT_change(3)<<std::endl;
      std::cout<<"group_id_:"<<group_id_<<std::endl;
      for(int i=0;i<v_group.size();i++)
      {
        std::cout<<"v_group:"<<v_group[i]<<std::endl;
      }
      if (flag_[j])
      {
        traj_optim[j] = formation_optim[j]->getMinJerkOptPtr()->getTraj();
      }
    }
    std::cout << "all planner!@@@!!" << std::endl;
  }

  /*****************************暂时手动给定leader以及组内编队,传进来在大编队里的id，传回小编队形状,小组成员编号（在大组的编号）以及小组id,以及小组数量************************/
  void get_txt(const int agent_number)
  {
    v_all.resize(agent_number);
    std::ifstream inputFile1("/home/zy/debug/1/group/face/kmeans_face_all.txt");
    std::ifstream inputFile2("/home/zy/debug/1/group/face/kmeans_face_group.txt");
    std::ifstream inputFile3("/home/zy/debug/1/group/face/kmeans_face_leader.txt");
    std::vector<Eigen::Vector3d> data_all;
    std::vector<std::vector<int>> data_group;
    double value;
    int value_group;
    Eigen::Vector3d value1;
    std::vector<int> value2;
    int flag_get_txt = 0;
    bool flag_group = false;
    while (inputFile1 >> value)
    {
      value1[flag_get_txt % 3] = value;
      if (flag_get_txt % 3 == 2)
      {
        data_all.push_back(value1);
        std::cout << "all:" << value1 << std::endl;
      }
      flag_get_txt = flag_get_txt + 1;
    }
    while (inputFile2 >> value_group)
    {
      if (value_group == 999 && !flag_group)
      {
        flag_group = true;
        data_group.push_back(value2);
        for (int i = 0; i < value2.size(); i++)
        {
          std::cout << "group:" << value2[i] << std::endl;
        }
        std::cout << "end!!!!" << std::endl;
        value2.clear();
      }
      else
      {
        flag_group = false;
        value2.push_back(value_group);
      }
    }
    flag_get_txt = 0;
    leader_id_get.resize(data_group.size());
    while (inputFile3 >> value)
    {
      leader_id_get[flag_get_txt] = value;
      flag_get_txt = flag_get_txt+1;
    }
    for(int i=0;i<leader_id_get.size();i++)
    {
      std::cout<<"leader_id_get:"<<leader_id_get[i]<<std::endl;
    }
    inputFile1.close();
    inputFile2.close();
    v_all = data_all;
    std::cout<<"vall_18::"<<v_all[18]<<std::endl;
    id_in_group.resize(data_group.size());
    id_in_group = data_group;
  }
  bool judge_is_leader(const int id_big_form)
  {
    bool flag_id_is_leader;
    flag_id_is_leader = false;
    for (int i = 0; i < leader_id_get.size(); i++)
    {
      if (id_big_form == leader_id_get[i])
      {
        flag_id_is_leader = true;
      }
    }
    return flag_id_is_leader;
  }
  int get_id_in_group(const int id_big_form)
  {
    int iiii;
    int group_id = form_to_group[id_big_form];
    std::cout << "group ip::" << group_id << std::endl;
    for (int i = 0; i < id_in_group[group_id].size(); i++)
    {
      std::cout << "iiiiiiiiiiii::" << i << std::endl;
      std::cout << "id_in_group[group_id]::" << id_in_group[group_id].at(i) << std::endl;
      if (id_big_form == id_in_group[group_id].at(i))
      {
        iiii = i;
      }
    }
    return iiii;
  }
  std::vector<int> get_every_group_num()
  {
    std::vector<int> every_num;
    every_num.resize(group_form.size());
    for (int i = 0; i < group_form.size(); i++)
    {
      every_num[i] = group_form[i].size();
    }
    return every_num;
  }
  inline std::vector<Eigen::Vector3d> get_v_group(int id_big_form)
  {
    std::vector<Eigen::Vector3d> v_group;
    v_group = group_form[id_big_form];
    return v_group;
  }
  inline int get_group_and_id(int id_big_form) // 传进来在大编队里的id
  {
    int group_id = form_to_group[id_big_form];
    return group_id;
  }
  int get_group_num()
  {
    int g_num;
    if (group_num_ > 0)
    {
      g_num = group_num_;
    }
    else
    {
      g_num = 1;
    }
    return g_num;
  }
  void group_swarm(bool m_or_n) // TODO (计算编队分组情况，得到每个小组的编队形状以及leader)   暂时手动分组!!!!
  {
    if (m_or_n)
    {
      /*********************************************20飞机******************************************************/
      std::cout << "ssssss yessssssss" << std::endl;
      std::vector<Eigen::Vector3d> group_n; // 某个组的编队形状
      // 计算得到分组数量
      group_num_ = id_in_group.size();
      // 计算得到每个组内的飞机id（不包括其他leader）
      form_to_group.resize(a_num_);
      for (int i = 0; i < group_num_; i++)
      {
        for (int j = 0; j < id_in_group[i].size(); j++)
        {
          form_to_group[id_in_group[i][j]] = i;
        }
      }

      // 计算每个编队的形状
      std::cout << "group des@^^^^^" << std::endl;
      group_form.resize(group_num_);
      std::cout << "group des@@@@@@" << std::endl;
      // 计算leader是哪些
      
      // leader_id_get = {10, 8, 11, 5};
      // leader_id_get = {16, 17, 18, 19};
      for (int k = 0; k < id_in_group.size(); k++)
      {
        std::cout << "kkkkkk^^^^^:" <<k<< std::endl;
        for (int i = 0; i < id_in_group[k].size(); i++)
        {
          int id_in_bigform = id_in_group[k][i];
          group_form[k].push_back(v_all[id_in_bigform]);
          std::cout << "v_all[id_in_bigform]:" <<v_all[id_in_bigform]<< std::endl;
        }
      }
    }
    else
    {
      std::cout << "ssssss noooooooo" << std::endl;
      std::vector<Eigen::Vector3d> group_n; // 某个组的编队形状
      // 计算得到分组数量
      group_num_ = 1;
      // 计算得到每个组内的飞机id（不包括其他leader）
      form_to_group.resize(a_num_);
      for (int i = 0; i < a_num_; i++)
      {
        form_to_group[i] = 0;
      }
      // 计算每个编队的形状
      std::cout << "group des@^^^^^" << std::endl;
      id_in_group.resize(group_num_);
      group_form.resize(group_num_);
      std::cout << "group des@@@@@@" << std::endl;
      // 计算leader是哪些
      leader_id_get.resize(group_num_);
      leader_id_get = {0};
      // 第一组情况
      id_in_group[0] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
      for (int i = 0; i < id_in_group[0].size(); i++)
      {
        int id_in_bigform = id_in_group[0][i];
        group_form[0].push_back(v_all[id_in_bigform]);
      }
      std::cout << "group des2$$$$" << std::endl;
    }
  }
  /*************************************暂时手动给定leader以及组内编队,传进来在大编队里的id，传回在小编队里的id以及小编队形状*********************************************/

  void outcome_vis(const double time_go)
  {
    std::vector<std::vector<Eigen::Vector3d>> traj_route;
    traj_route.resize(a_num_);
    std::vector<Eigen::Vector3d> swarm_g_;
    Eigen::Vector3d agent_pos_;
    // std::cout << "time go:" << time_go << std::endl;
    double total_time = traj_optim[id_chose_form].getTotalDuration();
    std::cout<<"T: 3.48216 2.91267 2.34978 3.66397::"<<std::endl;
    std::cout<<traj_optim[18].getPos(3.48)<<std::endl;
    std::cout<<traj_optim[18].getPos(2.9+3.482)<<std::endl;
    std::cout<<traj_optim[18].getPos(2.349+2.9+3.48)<<std::endl;
    std::cout<<"int ::"<<std::endl;
    std::cout<<traj_optim[18].getPos(0)<<std::endl;
    std::cout<<traj_optim[18].getPos(0.1)<<std::endl;
    std::cout<<traj_optim[18].getPos(0.4)<<std::endl;
    std::cout<<traj_optim[18].getPos(1)<<std::endl;
    for (int j = 0; j < a_num_; j++)
    {
      for (double i = 0; i < total_time; i = i + 0.4)
      {
        traj_route[j].push_back(traj_optim[j].getPos(i));
        // std::cout << "traj pos:" << traj_optim[id_chose_form].getPos(i) << std::endl;
      }
    }
    vis_rviz.visualize(traj_route, 18);
    for (int j = 0; j < a_num_; j++)
    {
      if (time_go > traj_optim[id_chose_form].getTotalDuration())
      {
        agent_pos_ = traj_optim[j].getPos(traj_optim[j].getTotalDuration());
        swarm_g_.push_back(agent_pos_);
      }
      else
      {
        agent_pos_ = traj_optim[j].getPos(time_go);
        swarm_g_.push_back(agent_pos_);
      }

    }
    formation_optim[0]->error_dist(swarm_g_, v_all);
    vis_rviz.swarm_visualize(swarm_g_);
    sensor_msgs::PointCloud2 esdf_vis;
    pcl::PointCloud<pcl::PointXYZ> point_obs_;
    formation_optim[0]->get_esdf(esdf_vis, point_obs_);
    vis_rviz.get_esdf_vis(esdf_vis, point_obs_);
  }

  void re_setup(const int agent_num)
  {
    a_num_ = agent_num;
    formation_optim.resize(agent_num);
    flag_.resize(agent_num);
    cstr_pts.resize(agent_num);
    headState.resize(agent_num);
    tailState.resize(agent_num);
    intState.resize(agent_num);
    head_pos.resize(agent_num);
    tail_pos.resize(agent_num);
    int_pos.resize(agent_num);
    int_pos1.resize(agent_num);
    int_pos2.resize(agent_num);
    head_vel = {0, 0, 0};
    head_acc = {0, 0, 0};
    tail_vel = {0, 0, 0};
    tail_acc = {0, 0, 0};
    for (int i = 0; i < agent_num; i++)
    {
      if(i==18)
      {
        std::cout<<"vall@@@@@@@@@@@@@@@@@@@@@@@@@@:"<<v_all[18]<<std::endl;
      }
      head_pos[i] = {v_all[i][0] - 4 +0.15*i  , v_all[i][1] -3.8-0.13*i, v_all[i][2]};
      tail_pos[i] = {v_all[i][0] + 16 - 4, v_all[i][1]-3.8, v_all[i][2]};

      int_pos[i] = {v_all[i][0] + 4 - 4, v_all[i][1]-3.8, v_all[i][2]};
      int_pos1[i] = {v_all[i][0] + 8 - 4, v_all[i][1]-3.8, v_all[i][2]};
      int_pos2[i] = {v_all[i][0] + 12 - 4, v_all[i][1]-3.8, v_all[i][2]};

      formation_optim[i].reset(new PolyTrajOptimizer);
      formation_optim[i]->setDroneId(i);
      formation_optim[i]->fisrt_planner_or_not(true);
      intState[i] << int_pos[i], int_pos1[i], int_pos2[i];
      headState[i] << head_pos[i], head_vel, head_acc;
      tailState[i] << tail_pos[i], tail_vel, tail_acc;
    }

    /************************************************************************************************************************/
    initT_.resize(4);
    initT_ << 2.7, 2.7, 2.7, 2.7;
    traj_optim.resize(agent_num);
    traj_global.resize(agent_num);
  }

private:
  // const int agent_num_get;
  Visualizer vis_rviz;
  std::vector<Eigen::Vector3d> v_all;                   // 整体编队的形状
  int group_num_;                                       // 分组数量
  std::vector<int> form_to_group;                       // 大编队里的id对应在第几组
  std::vector<std::vector<int>> id_in_group;            // 所有小组成员编号
  std::vector<std::vector<Eigen::Vector3d>> group_form; // 第某组的飞机编队
  std::vector<int> leader_id_get;                       // leader的id

  TrajContainer traj_;
  std::vector<TrajContainer> traj_real;
  int id_chose_form;
  int a_num_;

  std::vector<std::shared_ptr<PolyTrajOptimizer>> formation_optim;
  std::vector<poly_traj::Trajectory> traj_optim;
  std::shared_ptr<PolyTrajOptimizer> global_traj_opt;
  std::vector<poly_traj::Trajectory> traj_global;
  std::vector<bool> flag_;
  std::vector<Eigen::MatrixXd> cstr_pts;
  Eigen::VectorXd initT_;
  Eigen::VectorXd initT_2;
  std::vector<Eigen::Matrix<double, 3, 3>> headState, tailState;
  std::vector<Eigen::Matrix<double, 3, 3>> intState;
  std::vector<Eigen::Vector3d> head_pos;
  Eigen::Vector3d head_vel;
  Eigen::Vector3d head_acc;
  std::vector<Eigen::Vector3d> tail_pos;
  Eigen::Vector3d tail_vel;
  Eigen::Vector3d tail_acc;
  std::vector<Eigen::Vector3d> int_pos;
  std::vector<Eigen::Vector3d> int_pos1;
  std::vector<Eigen::Vector3d> int_pos2;

public:
  ego_planner_node(ros::NodeHandle &nh_) : vis_rviz(nh_){};
  ~ego_planner_node(){};

  void have_no_swarm()
  {
    double t_now_0 = 0;
    int piece_nums = 4;
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    int recv_id = 0;
    traj_.swarm_traj[recv_id].drone_id = recv_id;
    traj_.swarm_traj[recv_id].traj_id = 1;
    traj_.swarm_traj[recv_id].start_time = t_now_0;
    traj_.swarm_traj[recv_id].constraint_aware = 0;

    cMats[0].row(0) << -0.000411463, 0.00262856, -0.00482534, 0.00192251, 1.00824, -17.1803;
    cMats[0].row(1) << 0.000227876, -0.00163438, 0.0028976, 0.00414863, -0.00871011, -2.00708;
    cMats[0].row(2) << 0.000134042, -0.000122678, -0.00360272, 0.0081438, 0.0704743, 1.2073;
    dura[0] = 1.83508;
    cMats[1].row(0) << 0.000246434, -0.00114678, 0.000613024, 0.0030411, 1.00819, -15.3322;
    cMats[1].row(1) << -5.61373e-05, 0.00045648, -0.00142546, 0.00115986, 0.00831037, -2.00498;
    cMats[1].row(2) << -0.000223125, 0.00110721, 1.06812e-05, -0.00588538, 0.0685346, 1.34318;
    dura[1] = 1.86858;
    cMats[2].row(0) << -0.000368983, 0.00115562, 0.000646085, -0.00146881, 1.01107, -13.4421;
    cMats[2].row(1) << 1.95903e-05, -6.8004e-05, 2.63325e-05, -0.000930438, 0.00620453, -1.99042;
    cMats[2].row(2) << 0.000178149, -0.000977423, 0.000495723, 0.00281267, 0.0619462, 1.45918;
    dura[2] = 1.87471;
    cMats[3].row(0) << 0.00100066, -0.00230306, -0.00365615, 0.00222239, 1.02004, -11.54179;
    cMats[3].row(1) << -4.00929e-05, 0.000115627, 0.00020489, -0.000925602, 0.00241122, -1.98227;
    cMats[3].row(2) << -0.000123335, 0.000692466, -0.000572704, -0.0032727, 0.0629613, 1.58052;
    dura[3] = 1.89703;
    poly_traj::Trajectory trajectory0(dura, cMats);
    traj_.swarm_traj[recv_id].traj = trajectory0;
    traj_.swarm_traj[recv_id].duration = trajectory0.getTotalDuration();
    traj_.swarm_traj[recv_id].start_pos = trajectory0.getPos(0.0);
  }
  void deal_traj(int kkk, double b, double aa, double cc, int leader_id_z, int mm)
  {
    /*************************************************************00000000000************************************/
  }
};
#endif // EGO_PLANNER_NODE_H