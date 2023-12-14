
#include "m_planner_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mt_planner_node");
  ros::NodeHandle nh_;
  ros::Time::init();
  ego_planner_node m_planner;
  m_planner.do_planner(true, 3);
  return 0;
}
