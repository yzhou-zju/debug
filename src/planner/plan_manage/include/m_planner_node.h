#ifndef M_PLANNER_NODE_H
#define M_PLANNER_NODE_H
#include "visualizer.hpp"
#include <ros/ros.h>
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
    id_chose_form = 1;
    group_num_ = 1; // 初始化
    re_setup(agent_num_);
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
    std::vector<int> leader_id_;
    std::vector<Eigen::Vector3d> v_;

    bool m_or_not_get = false;
    bool flag_suss = true;
    traj_.swarm_traj.resize(1);
    have_no_swarm();
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/

    for (int i = 0; i < agent_num_; i++)
    {
      formation_optim[i]->setlog(i);
      formation_optim[i]->setParam(leader_id_, v_);
      formation_optim[i]->setSwarmTrajs(&traj_.swarm_traj);
      flag_[i] = formation_optim[i]->optimizeTrajectory_lbfgs(headState[i], tailState[i],
                                                              intState[i], initT_,
                                                              cstr_pts[i], true);
      if (!flag_[i])
      {
        flag_suss = false;
      }
      else
      {
        std::cout << "gena suss!!!!!!!!!!!" << std::endl;
        traj_optim[i] = formation_optim[i]->getMinJerkOptPtr()->getTraj();
      }
    }
    std::cout << "group des111" << std::endl;
    group_swarm(m_or_not_get);
    std::cout << "group des222222" << std::endl;
    planner_all_formation_once();
    std::cout << "group des33333" << std::endl;
    planner_all_formation_once();
    for (int i = 0; i < agent_num_; i++)
    {
      std::cout << "time total:" << traj_optim[i].getTotalDuration() << std::endl;
      std::cout << "pos:" << traj_optim[i].getPos(0) << std::endl;
      std::cout << "pos:" << traj_optim[i].getPos(2.5) << std::endl;
      std::cout << "pos:" << traj_optim[i].getPos(5) << std::endl;
      std::cout << "pos:" << traj_optim[i].getPos(7.5) << std::endl;
    }
  }

  void planner_all_formation_once()
  {
    std::vector<Eigen::Vector3d> v_group; // 小编队的形状
    std::vector<int> id_all;              // 小组成员编号（在大组的编号）
    std::vector<int> num_every_group;     // 每个小组内的数量
    int group_id_;                        // 在哪个小组
    int g_num;                            // 小组数量
    num_every_group = get_every_group_num();
    g_num = get_group_num();
    traj_real.resize(g_num);
    for (int i = 0; i < num_every_group.size(); i++)
    {
      traj_real[i].swarm_traj.resize(num_every_group[i]);
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
        std::cout << "iiiiiiiiiiii:" << i << std::endl;
        id_all = id_in_group[i];
        for (int k = 0; k < num_every_group[i]; k++)
        {
          traj_real[i].swarm_traj[k].drone_id = k;
          traj_real[i].swarm_traj[k].traj_id = k;
          traj_real[i].swarm_traj[k].start_time = t_now_0;
          traj_real[i].swarm_traj[k].constraint_aware = 0;
          traj_real[i].swarm_traj[k].traj = traj_optim[id_all[k]];
          traj_real[i].swarm_traj[k].duration = traj_optim[id_all[k]].getTotalDuration();
          traj_real[i].swarm_traj[k].start_pos = traj_optim[id_all[k]].getPos(0.0);
        }
        std::cout << "end iiiiii!!!" << std::endl;
      }
      /*********************************组内轨迹优化*********************************************/
      group_id_ = get_group_and_id(j);
      v_group = get_v_group(group_id_);
      recv_id = get_id_in_group(j);
      formation_optim[j]->setDroneId(recv_id);
      formation_optim[j]->fisrt_planner_or_not(false);
      formation_optim[j]->setParam(leader_id_, v_group);
      std::cout << "debug !!!!!!!666" << std::endl;
      formation_optim[j]->setSwarmTrajs(&traj_real[group_id_].swarm_traj);
      std::cout << "optimizeTrajectory_lbfgs" << std::endl;
      flag_[j] = formation_optim[j]->optimizeTrajectory_lbfgs(headState[j], tailState[j],
                                                              intState[j], initT_,
                                                              cstr_pts[j], true);
      if (flag_[j])
      {
        traj_optim[j] = formation_optim[j]->getMinJerkOptPtr()->getTraj();
      }
    }
    std::cout << "all planner!@@@!!" << std::endl;
  }

  /*****************************暂时手动给定leader以及组内编队,传进来在大编队里的id，传回小编队形状,小组成员编号（在大组的编号）以及小组id,以及小组数量************************/
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
    for (int i = 0; i < id_in_group[group_id].size(); i++)
    {
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
      std::vector<Eigen::Vector3d> group_n; // 某个组的编队形状
      // 计算得到分组数量
      group_num_ = 4;
      // 计算得到每个组内的飞机id（不包括其他leader）
      form_to_group.resize(a_num_);
      form_to_group[0] = 0;
      form_to_group[1] = 0;
      form_to_group[2] = 1;
      form_to_group[3] = 1;
      form_to_group[4] = 0;
      form_to_group[5] = 0;
      form_to_group[6] = 1;
      form_to_group[7] = 1;
      form_to_group[8] = 2;
      form_to_group[9] = 2;
      form_to_group[10] = 3;
      form_to_group[11] = 3;
      form_to_group[12] = 2;
      form_to_group[13] = 2;
      form_to_group[14] = 3;
      form_to_group[15] = 3;
      // 计算每个编队的形状
      std::cout << "group des@^^^^^" << std::endl;
      id_in_group.resize(group_num_);
      group_form.resize(group_num_);
      std::cout << "group des@@@@@@" << std::endl;
      // 计算leader是哪些
      leader_id_get.resize(group_num_);
      leader_id_get = {1, 3, 9, 11};
      // 第一组情况
      id_in_group[0] = {0, 1, 4, 5, 3, 9, 11};
      for (int i = 0; i < id_in_group[0].size(); i++)
      {
        int id_in_bigform = id_in_group[0][i];
        group_form[0].push_back(v_all[id_in_bigform]);
      }
      std::cout << "group des2$$$$" << std::endl;
      // 第二组情况
      id_in_group[1] = {2, 3, 6, 7, 1, 9, 11};
      for (int i = 0; i < id_in_group[1].size(); i++)
      {
        int id_in_bigform = id_in_group[1][i];
        group_form[1].push_back(v_all[id_in_bigform]);
      }
      // 第三组情况
      id_in_group[2] = {8, 9, 12, 13, 1, 3, 11};
      for (int i = 0; i < id_in_group[2].size(); i++)
      {
        int id_in_bigform = id_in_group[2][i];
        group_form[2].push_back(v_all[id_in_bigform]);
      }
      // 第四组情况
      id_in_group[3] = {10, 11, 14, 15, 1, 3, 9};
      for (int i = 0; i < id_in_group[3].size(); i++)
      {
        int id_in_bigform = id_in_group[3][i];
        group_form[3].push_back(v_all[id_in_bigform]);
      }
    }
    else
    {
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
      leader_id_get = {1};
      // 第一组情况
      id_in_group[0] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
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
    std::vector<Eigen::Vector3d> traj_route;
    std::vector<Eigen::Vector3d> swarm_g_;
    Eigen::Vector3d agent_pos_;
    // std::cout << "time go:" << time_go << std::endl;
    double total_time = traj_optim[id_chose_form].getTotalDuration();
    for (double i = 0; i < total_time; i = i + 1.0)
    {
      traj_route.push_back(traj_optim[id_chose_form].getPos(i));
      // std::cout << "traj pos:" << traj_optim[id_chose_form].getPos(i) << std::endl;
    }
    vis_rviz.visualize(traj_route, 0);

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

      // std::cout << "agent_pos_:" << agent_pos_ << std::endl;
    }
    formation_optim[0]->error_dist(swarm_g_, v_all);
    vis_rviz.swarm_visualize(swarm_g_);
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

    int flag_num = 0;
    for (int i = 0; i < agent_num / 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        if (flag_num != id_chose_form)
        {
          head_pos[flag_num] = {-2 * i, j * 2, 1};
          tail_pos[flag_num] = {-2 * i + 10, j * 2, 1};
        }
        else
        {
          head_pos[flag_num] = {-2 * i + 1.5, j * 2, 1};
          tail_pos[flag_num] = {-2 * i + 10, j * 2, 1};
        }

        int_pos[flag_num] = {-2 * i + 2.5, j * 2, 1};
        int_pos1[flag_num] = {-2 * i + 5.0, j * 2, 1};
        int_pos2[flag_num] = {-2 * i + 7.5, j * 2, 1};

        formation_optim[flag_num].reset(new PolyTrajOptimizer);
        formation_optim[flag_num]->setDroneId(flag_num);
        formation_optim[flag_num]->fisrt_planner_or_not(true);
        intState[flag_num] << int_pos[flag_num], int_pos1[flag_num], int_pos2[flag_num];
        headState[flag_num] << head_pos[flag_num], head_vel, head_acc;
        tailState[flag_num] << tail_pos[flag_num], tail_vel, tail_acc;

        flag_num = flag_num + 1;
      }
    }

    initT_.resize(4);
    initT_ << 2.5, 2.5, 2.5, 2.5;
    traj_optim.resize(agent_num);

    set_v_all(agent_num);
  }
  /*****************设置整体编队形状*************************/
  void set_v_all(const int agent_num_set)
  {
    v_all.resize(16);
    int id_form = 0;
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        v_all[id_form] = {i * 2, j * 2, 1};
        id_form = id_form + 1;
      }
    }
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