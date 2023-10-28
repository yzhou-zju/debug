/*** 
 * @Author: ztr
 * @Date: 2022-05-12 00:00:43
 * @LastEditTime: 2022-05-12 00:00:43
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /Pipline-Swarm-Formation/src/planner/plan_manage/src/ego_planner_node.cpp
 */
#include "ego_planner_node.h"


int main(int argc, char **argv)
{
  ros::Time::init();
  PolyTrajOptimizer::Ptr formation_optim;
  formation_optim.reset(new PolyTrajOptimizer);
  formation_optim->setDroneId(1);
  /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
  /*******一共九组，一组8个飞机，默认leader为0 ，8，16，24，32，40，48，56，64号飞机******/
  std::vector<int> leader_id;
  leader_id.resize(8);
  /*drone_*/
  leader_id.at(0) = 8;                              /*from 8 to 15*/
  leader_id.at(1) = 16;                           /*from 16 to 23*/
  leader_id.at(2) = 24;                            /*from 24 to 31*/
  leader_id.at(3) = 32;                            /*from 32 to 39*/
  leader_id.at(4) = 40;                            /*from 40 to 47*/
  leader_id.at(5) = 48;                            /*from 48 to 55*/
  leader_id.at(6) = 56;                            /*from 56 to 63*/
  leader_id.at(7) = 64;                            /*from 64 to 71*/

  bool m_or_not_get = true;
  // bool m_or_not_get = false;
  /*********************************************************m_or_not_get 表示是否分组  true表示分组******************************************************************************/
  formation_optim->setParam(m_or_not_get,leader_id);
  if(m_or_not_get){
  traj_.swarm_traj.resize(16);
  deal_traj(0,0.0,0,0,-1,-1);
  deal_traj(0,2.0,1,0,leader_id.at(0),8);
  deal_traj(0,-2.0,0,0,leader_id.at(1),9);
  deal_traj(0,4.0,0,0,leader_id.at(2),10);
  deal_traj(0,-4.0,0,0,leader_id.at(3),11);
  deal_traj(0,-6.0,0,0,leader_id.at(4),12);
  deal_traj(0,-8.0,0,0,leader_id.at(5),13);
  deal_traj(0,6.0,0,0,leader_id.at(6),14);
  deal_traj(0,8.0,0,0,leader_id.at(7),15);

  }else{
  traj_.swarm_traj.resize(72);
  deal_traj(0,0.0,0,0,-1,-1);
  deal_traj(8,2.0,0,0,-1,-1);
  deal_traj(16,-2.0,0,0,-1,-1);
  deal_traj(24,4.0,0,0,-1,-1);
  deal_traj(32,-4.0,0,0,-1,-1);
  deal_traj(40,-6.0,0,0,-1,-1);
  deal_traj(48,-8.0,0,0,-1,-1);
  deal_traj(56,6.0,0,0,-1,-1);
  deal_traj(64,8.0,0,0,-1,-1);
  }

  formation_optim->setSwarmTrajs(&traj_.swarm_traj);
  
  bool flag_;
  Eigen::MatrixXd cstr_pts;
  Eigen::VectorXd initT_;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  // Eigen::MatrixXd intState;
  Eigen::Matrix<double, 3, 3> intState;
  Eigen::Vector3d head_pos;
  Eigen::Vector3d head_vel;
  Eigen::Vector3d head_acc;
  Eigen::Vector3d tail_pos;
  Eigen::Vector3d tail_vel;
  Eigen::Vector3d tail_acc;
  Eigen::Vector3d int_pos;
  Eigen::Vector3d int_vel;
  Eigen::Vector3d int_acc;
  head_pos = {-17.1896,-2.01374,1.2061};
  head_vel = {1.00248,0.0031434,0.0801646 };
  head_acc = {0.00153082,0.0194509,0.0231634};
  tail_pos = {-9.62895, -1.97912,1.69021};
  tail_vel = {0.990905,0.00167276 ,0.0552846 };
  tail_acc = {-1.11022e-16,3.46945e-18,1.04083e-17};

  int_pos = {-15.3564,-2.00246,1.34311};
  int_vel = {-13.454,-1.99745,1.45262};
  int_acc = {-11.5558,-1.98311,1.58101};

  intState <<  int_pos,int_vel,int_acc;

  headState << head_pos, head_vel, head_acc;
  tailState << tail_pos, tail_vel, tail_acc;
  initT_.resize(4);
  initT_<<1.87849, 1.87849, 1.87849, 1.87849;
  poly_traj::Trajectory traj_optim;
  flag_ =  formation_optim->optimizeTrajectory_lbfgs(headState, tailState,
                                                            intState, initT_,
                                                            cstr_pts, true);
  std::cout<<"f*** you1!!!!!!!!!!"<<std::endl;
  if(flag_)
  {
    std::cout<<"f*** you all!!!!!!!!!!"<<std::endl;
    traj_optim = formation_optim->getMinJerkOptPtr()->getTraj();
    double error_dist_ = formation_optim->error_dist(traj_optim);
    std::cout<<"error_dist_:"<<error_dist_<<std::endl;
  }else{
    std::cout<<"f*** you!!!!!!!!!!"<<std::endl;
  }
  return 0;
}
