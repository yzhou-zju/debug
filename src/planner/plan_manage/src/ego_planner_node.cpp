
#include "m_planner_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mt_planner_node");
  ros::NodeHandle nh_;
  ros::Time::init();
  ego_planner_node m_planner(nh_);
  m_planner.do_planner(true, 20);
  ros::Time t_old;
  ros::Time t_0 = ros::Time::now();
  t_old = t_0;
  ros::Rate lr(10);
  std::cout<<"vis!!!!!!!!!!!"<<std::endl;
  while (ros::ok())
  {
    ros::Time t_1 = ros::Time::now();
    double t_go = (t_1 - t_0).toSec();
    // if ((t_1 - t_old).toSec() > 2.5)
    // {
    //   m_planner.planner_all_formation_once(t_go, t_1, t_0);
    //   m_planner.outcome_vis(0);
    //   t_0 = ros::Time::now();
    //   t_old = t_0;
    // }
    // else
    // {
    //   m_planner.outcome_vis(t_go);
    // }

    // mt_planner.process();
    m_planner.outcome_vis(t_go);
    ros::spinOnce();
    lr.sleep();
  }
  return 0;
}
