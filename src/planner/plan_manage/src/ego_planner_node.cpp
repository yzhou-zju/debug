/*** 
 * @Author: ztr
 * @Date: 2022-05-12 00:00:43
 * @LastEditTime: 2022-05-12 00:00:43
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /Pipline-Swarm-Formation/src/planner/plan_manage/src/ego_planner_node.cpp
 */
#include "ego_planner_node.h"
using namespace ego_planner;
using namespace poly_traj;
int main(int argc, char **argv)
{
  ros::Time::init();
  TrajContainer traj_;
  for(int i=0;i<8;i++)
  {
    LocalTrajData blank;
    blank.drone_id = -1; //表示还没接收到轨迹
    traj_.swarm_traj.push_back(blank);
  }
  // PolyTrajOptimizer formation_optim;
  PolyTrajOptimizer::Ptr formation_optim;
  formation_optim.reset(new PolyTrajOptimizer);
  formation_optim->setDroneId(0);
  formation_optim->setParam();
  formation_optim->setSwarmTrajs(&traj_.swarm_traj);
  bool flag_;
  Eigen::MatrixXd cstr_pts;
  Eigen::VectorXd initT_;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  // Eigen::MatrixXd intState;
  Eigen::Matrix<double, 3, 1> intState;
  Eigen::Vector3d head_pos;
  Eigen::Vector3d head_vel;
  Eigen::Vector3d head_acc;
  Eigen::Vector3d tail_pos;
  Eigen::Vector3d tail_vel;
  Eigen::Vector3d tail_acc;
  Eigen::Vector3d int_pos;
  Eigen::Vector3d int_vel;
  Eigen::Vector3d int_acc;
  head_pos = {0,0,0};
  head_vel = {0,0,0};
  head_acc = {0,0,0};
  tail_pos = {10,10,10};
  tail_vel = {0,0,0};
  tail_acc = {0,0,0};
  int_pos = {5,5,5};
  int_vel = {1,1,1};
  int_acc = {1,1,1};
  headState << head_pos, head_vel, head_acc;
  tailState << tail_pos, tail_vel, tail_acc;
  intState <<  int_pos;
  initT_.resize(2);
  initT_<<10,10;
  poly_traj::Trajectory traj_optim;
  flag_ =  formation_optim->optimizeTrajectory_lbfgs(headState, tailState,
                                                            intState, initT_,
                                                            cstr_pts, false);
  std::cout<<"fuck you1!!!!!!!!!!"<<std::endl;
  if(flag_)
  {
    std::cout<<"fuck you all!!!!!!!!!!"<<std::endl;
    traj_optim = formation_optim->getMinJerkOptPtr()->getTraj();
  }else{
    std::cout<<"fuck you!!!!!!!!!!"<<std::endl;
  }
  return 0;
}
