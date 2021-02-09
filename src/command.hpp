#ifndef COMMAND_HPP
#define COMMAND_HPP
#include <vector>
#include <memory>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// subclass of NodeHandle
class Command_node : public ros::NodeHandle{
private:
  bool enable_static_map_;
  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist vel_msg_;

public:
  Command_node(const std::string & ns = std::string());
  // control the robot to visit points
  void visit_points(std::vector<std::array<double,2>> path);
  // control the robot to follow quadratic bezier curves
  void follow_quadratic_bezier_path(std::vector<std::array<double,2>> path);
  // return a path (list of points) from the current position to the goal
  // set quadratic_bezier to true to get a list of control points of quadratic bezier curves
  std::vector<std::array<double, 2>> path_from_rrt(std::array<double, 2> goal, bool quadratic_bezier=false);
  // orthogonal projection quadratic bezier curve all-trigonometric algorithm
  // return t which is supposed to be in [0,1], m is the point to project on the curve
  double opqbcata(std::array<double, 2> p0, std::array<double, 2> p1, std::array<double, 2> p2, std::array<double, 2> m);
};


template<typename T> // dot product for 2D
inline double dot2(T a, T b){return a[0]*b[0]+a[1]*b[1];}

template<typename T> // distance or norm for 2D
inline double dist2(T a, T b={0,0}){return std::sqrt(std::pow(a[0]-b[0],2) + std::pow(a[1]-b[1],2));}

template<typename T> // point at position t of quaratic bezier curve a,b,c in 2D
inline T qb2(T a, T b, T c, double t){
  return T {std::pow(1-t,2)*a[0] + 2*(1-t)*t*b[0] + std::pow(t,2)*c[0],
            std::pow(1-t,2)*a[1] + 2*(1-t)*t*b[1] + std::pow(t,2)*c[1]};
}

template<typename T> // derivate at position t
inline T qbp2(T a, T b, T c, double t){
  return T {2*(1-t)*(b[0]-a[0]) + 2*t*(c[0]-b[0]),2*(1-t)*(b[1]-a[1]) + 2*t*(c[1]-b[1])};
}

#endif
