#include "ros/ros.h"
#include "command.hpp"

#include <vector>
#include <unistd.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "rrt");
  Command_node n("~");
  sleep(2); // wait to let the map load
  std::array<double, 2> goal = {-15, -7};
  //n.visit_points(n.path_from_rrt(goal));
  n.follow_quadratic_bezier_path(n.path_from_rrt(goal, true));
  //n.visit_points(std::vector<std::array<double, 2>>{{-1,3.5}, {-3,3.5}, {-4,-8.5}, {-6,-8.5}, {-8,3.5}, {-15,-7}});
  //n.follow_quadratic_bezier_path(std::vector<std::array<double, 2>>{{0.00, 0.00},{-2.48, 6.10},{-3.04, 1.81},{-3.36, -2.63},{-4.24, -6.54},{-5.09, -10.48},{-5.59, -6.50},{-6.03, -2.43},{-6.35, 1.37},{-6.59, 5.26},{-8.47, 2.11},{-10.26, -1.06}, {-15.00, -7.00}});
  std::cout<<"end" <<std::endl;
}
