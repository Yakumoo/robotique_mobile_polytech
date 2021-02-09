#include "ros/ros.h"
#include "RRT.hpp"
#include "nav_msgs/GetMap.h"
#include <vector>
#include <unistd.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "rrt");
  ros::NodeHandle n;
  //sleep(1);
  // get the map
  ros::ServiceClient client =n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid map;
  if (client.call(srv)){
    ROS_INFO("Message recu");
    map=std::move(srv.response.map);
  }else{
    ROS_ERROR("Failed to call service static_map");
    return 1;
  }

  // set up parameters
  std::array<double, 2> start={0,0}, goal={-15,-7}, origin={map.info.origin.position.x, map.info.origin.position.y};
  double res = map.info.resolution, dq=1, radius=0.3;
  std::vector<std::vector<int>> map_vec(map.info.height, std::vector<int>(map.info.width, 0));
  for (size_t i = 0; i < map.info.height; i++) {
    for (size_t j = 0; j < map.info.width; j++) {
      map_vec[i][j] = map.data[i*map.info.width+j];
    }
  }
  printf("height %.d width %d resolution %.3f\n", map.info.height, map.info.width, map.info.resolution);
  printf("xrange(%.3f %.3f) yrange(%.3f %.3f)\n", map.info.origin.position.x, map.info.origin.position.x+map.info.width*map.info.resolution, map.info.origin.position.y, map.info.origin.position.y+map.info.height*map.info.resolution);
  RRT rrt(map_vec, origin, res, dq, radius);
  std::cout<<"start build"<<std::endl;
  std::vector<std::array<double, 2>> path = rrt.connect(start, goal, ros::ok);

  for (auto const& e : path) { // print the path
    printf("(%.2f, %.2f) ", e[0], e[1]);
  }
  std::cout << std::endl;
  path = rrt.safe_quadratic_bezier_path(path);
  for (auto const& e : path) { // print the path
    printf("(%.2f, %.2f) ", e[0], e[1]);
  }
  //rrt.safe_quadratic_bezier_path(std::vector<std::array<double, 2>> {{0,0}, {-2,3.5}, {-5,-8.5}, {-7,3.5}, {-15,-7}});
  std::cout<<"end" <<std::endl;
}
