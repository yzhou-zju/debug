
#include "m_planner_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh_("~");
  ros::Time::init();
  int a_number_;
  int randow_number_;
  bool get_plan_ok_ = false;
  nh_.param("agent_number_", a_number_, 1);
  nh_.param("randow_number", randow_number_, 1);

  ego_planner_node m_planner(nh_);
  for (int i = 1; i < randow_number_+1; i++)
  {
    m_planner.do_planner(true, a_number_,i);
    get_plan_ok_ = false;
    ros::Time t_old;
    ros::Time t_0 = ros::Time::now();
    t_old = t_0;
    ros::Rate lr(10);
    while (ros::ok() && !get_plan_ok_)
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
      get_plan_ok_ = m_planner.get_plan_ok();
      ros::spinOnce();
      lr.sleep();
    }

    m_planner.close_zy_log();
  }

  return 0;
}
