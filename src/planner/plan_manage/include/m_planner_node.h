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
  do_planner(bool m_or_not_get_, const int agent_num_)
  {
    id_chose_form = 12;
    re_setup(agent_num_);
    fack_traj();
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
    /*******一共九组，一组8个飞机，默认leader为0 ，8，16，24，32，40，48，56，64号飞机******/
    std::vector<int> leader_id;
    leader_id.resize(8);
    /*drone_*/
    leader_id.at(0) = 8;  /*from 8 to 15*/
    leader_id.at(1) = 16; /*from 16 to 23*/
    leader_id.at(2) = 24; /*from 24 to 31*/
    leader_id.at(3) = 32; /*from 32 to 39*/
    leader_id.at(4) = 40; /*from 40 to 47*/
    leader_id.at(5) = 48; /*from 48 to 55*/
    leader_id.at(6) = 56; /*from 56 to 63*/
    leader_id.at(7) = 64; /*from 64 to 71*/

    // bool m_or_not_get = true;
    bool m_or_not_get = true;
    bool no_swarm = true;
    /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
    for (int i = 0; i < agent_num_; i++)
    {
      formation_optim[i]->setParam(m_or_not_get, leader_id);
    }

    if (no_swarm)
    {
      traj_.swarm_traj.resize(1);
      have_no_swarm();
    }

    bool flag_suss = true;
    std::cout << "@@@@@@3!!!!:" << flag_suss << std::endl;
    for (int i = 0; i < agent_num_; i++)
    {
      if (i != id_chose_form) /***********************************************/
      {
        std::cout << "@@####@@#!!!:" << i << std::endl;
        formation_optim[i]->setSwarmTrajs(&traj_.swarm_traj);
        std::cout << "@@##$$$$#!!!:" << i << std::endl;
        flag_[i] = formation_optim[i]->optimizeTrajectory_lbfgs(headState[i], tailState[i],
                                                                intState[i], initT_,
                                                                cstr_pts[i], true);
        std::cout << "@@##&&&&:" << i << std::endl;
        if (!flag_[i])
        {
          flag_suss = false;
        }
        else
        {
          traj_optim[i] = formation_optim[i]->getMinJerkOptPtr()->getTraj();
        }
      }
    }
    std::cout << "@@#####!!!:" << flag_suss << std::endl;
    if (flag_suss)
    {
      traj_real.swarm_traj.resize(agent_num_);
      std::cout << "@@@@@@!!!!" << std::endl;
      test3agent(traj_optim);
      std::cout << "@@@@@@1!!!!" << std::endl;
      formation_optim[id_chose_form]->setSwarmTrajs(&traj_real.swarm_traj);
      std::cout << "@@@@@@2!!!!" << std::endl;
      flag_[id_chose_form] = formation_optim[id_chose_form]->optimizeTrajectory_lbfgs(headState[id_chose_form], tailState[id_chose_form],
                                                                                      intState[id_chose_form], initT_,
                                                                                      cstr_pts[id_chose_form], true);
      std::cout << "@@@@@@3!!!!" << std::endl;
    }

    if (flag_[id_chose_form])
    {
      for (int i = 0; i < agent_num_; i++)
      {
        if (i != id_chose_form)
        {
          std::cout << "time total:" << traj_optim[i].getTotalDuration() << std::endl;
          std::cout << "pos:" << traj_optim[i].getPos(0) << std::endl;
          std::cout << "pos:" << traj_optim[i].getPos(2.5) << std::endl;
          std::cout << "pos:" << traj_optim[i].getPos(5) << std::endl;
          std::cout << "pos:" << traj_optim[i].getPos(7.5) << std::endl;
        }
      }

      traj_optim[id_chose_form] = formation_optim[id_chose_form]->getMinJerkOptPtr()->getTraj();
      std::cout << "id_chose_form:" << id_chose_form << std::endl;
      std::cout << "time total2:" << traj_optim[id_chose_form].getTotalDuration() << std::endl;
      std::cout << "pos2:" << traj_optim[id_chose_form].getPos(0) << std::endl;
      std::cout << "pos2:" << traj_optim[id_chose_form].getPos(2.5) << std::endl;
      std::cout << "pos2:" << traj_optim[id_chose_form].getPos(5) << std::endl;
      std::cout << "pos2:" << traj_optim[id_chose_form].getPos(7.5) << std::endl;
    }
    planner_all_formation_once();
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
    void planner_all_dyn()
  {
  
  }

  void planner_all_formation_once()
  {
    for (int j = 0; j < a_num_; j++)
    {
      double t_now_0 = 0;
      int piece_nums = 4;
      int recv_id;
      for (int i = 0; i < traj_optim.size(); i++)
      {
        recv_id = i;
        traj_real.swarm_traj[recv_id].drone_id = recv_id;
        traj_real.swarm_traj[recv_id].traj_id = i;
        traj_real.swarm_traj[recv_id].start_time = t_now_0;
        traj_real.swarm_traj[recv_id].constraint_aware = 0;
        traj_real.swarm_traj[recv_id].traj = traj_optim[i];
        traj_real.swarm_traj[recv_id].duration = traj_optim[i].getTotalDuration();
        traj_real.swarm_traj[recv_id].start_pos = traj_optim[i].getPos(0.0);
      }
      formation_optim[j]->setDroneId(j);
      formation_optim[j]->fisrt_planner_or_not(false);
      formation_optim[j]->setSwarmTrajs(&traj_real.swarm_traj);
      flag_[j] = formation_optim[j]->optimizeTrajectory_lbfgs(headState[j], tailState[j],
                                                              intState[j], initT_,
                                                              cstr_pts[j], true);
      if (flag_[j])
      {
        traj_optim[j] = formation_optim[j]->getMinJerkOptPtr()->getTraj();
      }
    }
  }
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
          // std::cout << "$^%^%*^&*^*(99999999999((*(&*(&*&&())(*)))))" << std::endl;
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
  }

private:
  // const int agent_num_get;
  Visualizer vis_rviz;

  TrajContainer traj_;
  TrajContainer traj_real;
  int id_chose_form;
  int a_num_;
  poly_traj::Trajectory traj_fack;

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

  void test3agent(const std::vector<poly_traj::Trajectory> traj_test)
  {
    double t_now_0 = 0;
    int piece_nums = 4;
    int recv_id;
    std::cout << "traj size()::::" << traj_test.size() << std::endl;
    for (int i = 0; i < traj_test.size(); i++)
    {
      if (i != id_chose_form)
      {
        recv_id = i;
        traj_real.swarm_traj[recv_id].drone_id = recv_id;
        traj_real.swarm_traj[recv_id].traj_id = i;
        traj_real.swarm_traj[recv_id].start_time = t_now_0;
        traj_real.swarm_traj[recv_id].constraint_aware = 0;
        traj_real.swarm_traj[recv_id].traj = traj_test[i];
        traj_real.swarm_traj[recv_id].duration = traj_test[i].getTotalDuration();
        traj_real.swarm_traj[recv_id].start_pos = traj_test[i].getPos(0.0);
      }
      else
      {
        recv_id = i;
        traj_real.swarm_traj[recv_id].drone_id = recv_id;
        traj_real.swarm_traj[recv_id].traj_id = id_chose_form;
        traj_real.swarm_traj[recv_id].start_time = t_now_0;
        traj_real.swarm_traj[recv_id].constraint_aware = 0;

        traj_real.swarm_traj[recv_id].traj = traj_fack;
        traj_real.swarm_traj[recv_id].duration = traj_fack.getTotalDuration();
        traj_real.swarm_traj[recv_id].start_pos = traj_fack.getPos(0.0);
      }
    }
    std::cout << "traj size()::::" << std::endl;
  }
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
  void fack_traj()
  {
    int piece_nums = 4;
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
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
    traj_fack = trajectory0;
  }
};
#endif // EGO_PLANNER_NODE_H