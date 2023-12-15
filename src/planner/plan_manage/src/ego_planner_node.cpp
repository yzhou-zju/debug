
#include "m_planner_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mt_planner_node");
  ros::NodeHandle nh_;
  ros::Time::init();
  ego_planner_node m_planner(nh_);
  m_planner.do_planner(true, 16);

  ros::Time t_0 = ros::Time::now();
  ros::Rate lr(10);
  while (ros::ok())
  {
    ros::Time t_1 = ros::Time::now();
    double t_go = (t_1 - t_0).toSec();
    m_planner.outcome_vis(t_go);
    // mt_planner.process();
    ros::spinOnce();
    lr.sleep();
  }
  return 0;
}
