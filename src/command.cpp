#include "command.hpp"
#include "RRT.hpp"
#include "nav_msgs/GetMap.h"
#include <tf/transform_listener.h>
#include <chrono>
#include <cmath>

#define PI 3.14159265358979323846264338327950288419716939937510582
#define PI2 1.57079632679489661923


// set the ros param enable_static_map to true to use static_map otherwise dynamic_map will be called by default
Command_node::Command_node(const std::string & ns):
NodeHandle(ns), velocity_publisher_(advertise<geometry_msgs::Twist>("/cmd_vel", 10)){
  getParam("enable_static_map", enable_static_map_);
}

void Command_node::visit_points(std::vector<std::array<double,2>> path){
  if (path.size()==0) return;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::array<double, 2> v, p;
  int i=0;
  double l=0.2, roll, pitch, yaw, k=0.5;
  for (ros::Rate rate(100); ok(); rate.sleep()) {
    try{
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
      transform.getBasis().getRPY(roll, pitch, yaw); // angles
      p[0] = transform.getOrigin().x()+l*std::cos(yaw); // point p
      p[1] = transform.getOrigin().y()+l*std::sin(yaw);
      if (dist2(p,path[i])<0.1 && ++i==path.size()) break; // end
      v[0] = -k*(p[0]-path[i][0]); // error
      v[1] = -k*(p[1]-path[i][1]);
      vel_msg_.linear.x=(std::cos(yaw)*v[0] + std::sin(yaw)*v[1]); // convert to the command
      vel_msg_.angular.z=(-std::sin(yaw)*v[0] + std::cos(yaw)*v[1])/l;
      velocity_publisher_.publish(vel_msg_);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }
}



void Command_node::follow_quadratic_bezier_path(std::vector<std::array<double,2>> path){
  if (path.size()==0) return;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::array<double, 2> v, p, beginning=qb2(path[0],path[1],path[2], 0.1), projection, normal, z;
  int i=1;
  double l=0.3, roll, pitch, yaw, k=0.1, angle, t, tn, norm, d;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(), now=std::chrono::steady_clock::now();
  vel_msg_.linear.x = 0;
  // let the robot orientate for 2s
  for (ros::Rate rate(20); ok() && std::chrono::duration_cast<std::chrono::duration<double>>(now - begin).count()<3; rate.sleep()) {
    now=std::chrono::steady_clock::now();
    try{
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
      angle = std::atan2(beginning[1]-transform.getOrigin().y(), beginning[0]-transform.getOrigin().x());
      transform.getBasis().getRPY(roll, pitch, yaw);
      vel_msg_.angular.z = 2*(angle - yaw);
      velocity_publisher_.publish(vel_msg_);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      begin = now;
    }
  }
  // go !
  for (ros::Rate rate(20); ok(); rate.sleep()) {
    try{
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
      transform.getBasis().getRPY(roll, pitch, yaw); // angles
      p[0] = transform.getOrigin().x()+l*std::cos(yaw); // point p
      p[1] = transform.getOrigin().y()+l*std::sin(yaw);
      tn = opqbcata(path[i-1], path[i], path[i+1], p);
      if (tn > -0.05 && tn <= 1.05) {
        t = tn;
        projection = qb2(path[i-1], path[i], path[i+1], t);
        normal = qbp2(path[i-1], path[i], path[i+1], t); // this is actually the tangent
        angle = yaw - std::atan2(normal[1], normal[0]); // θe
        norm = dist2(normal);
        normal = {normal[1]/norm, -normal[0]/norm}; // convert to normalized normal
        d = dot2(normal, std::array<double,2> {p[0]-projection[0], p[1]-projection[1]});
        vel_msg_.linear.x = 1; // set the speed here
        vel_msg_.angular.z = - (std::tan(angle)/l + k*d/std::cos(angle)) * vel_msg_.linear.x;
        if (i<path.size()-2
            && (tn = opqbcata(path[i+1], path[i+2], path[i+3], p))> -0.05
            && tn<=1
            && dist2(p, qb2(path[i+1], path[i+2], path[i+3], tn)) < std::abs(d)){ // compute again the distance to compare
          i+=2; // we are closer to the next curve so we increment
          continue;
        }else if (dist2(p, path[path.size()-1]) < 0.3) break; // if end
      }else if (i<path.size()-2 && opqbcata(path[i+1], path[i+2], path[i+3], p) > -0.05) { // try next if the current failed
        i+=2;
        continue;
      }else if (i==path.size()-2 && dist2(p, path[path.size()-1])<0.3) break;
      else{ // swith to visit_points control and the target is the next point
        if (t>=0 && t<0.5) { // where was the last position
          v[0] = -k*(p[0]-path[i][0]/2 + path[i-1][0] + path[i+1][0]);
          v[1] = -k*(p[1]-path[i][1]/2 + path[i-1][1] + path[i+1][1]);
        }else{
          v[0] = -k*(p[0]-path[i+1][0]);
          v[1] = -k*(p[1]-path[i+1][1]);
        }
        vel_msg_.linear.x = (std::cos(yaw)*v[0] + std::sin(yaw)*v[1]); // convert to the command
        vel_msg_.angular.z = (-std::sin(yaw)*v[0] + std::cos(yaw)*v[1])/l;
      }
      velocity_publisher_.publish(vel_msg_);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }
  vel_msg_.linear.x = 0;
  vel_msg_.angular.z = 0;
  begin = std::chrono::steady_clock::now();
  now = std::chrono::steady_clock::now();
  for (ros::Rate rate(20); ok() && std::chrono::duration_cast<std::chrono::duration<double>>(now - begin).count()<1; rate.sleep()) {
    velocity_publisher_.publish(vel_msg_);
    now = std::chrono::steady_clock::now();
  }
}

// http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
// https://quarticequations.com/Cubic.pdf
double Command_node::opqbcata(std::array<double, 2> p0, std::array<double, 2> p1, std::array<double, 2> p2, std::array<double, 2> m){
  std::array<double, 2> A = {p1[0]-p0[0], p1[1]-p0[1]}, B={p2[0]-p1[0]-A[0], p2[1]-p1[1]-A[1]}, M={p0[0]-m[0], p0[1]-m[1]};
  double a = dot2(B,B), b = 3*dot2(A,B)/a, c = (2*dot2(A,A)+dot2(M,B))/a, d=dot2(M,A)/a, t=-1,
    q = c/3 -b*b/9, r = (c*b-3*d)/6 - b*b*b/27, gamma, chi;
  if (r*r + q*q*q > 0) {
    if (q < 0) {
      gamma = std::asin(std::max(-1., std::min(std::pow(-q, 3/2.)/r, 1.)));
      chi = std::atan(std::cbrt(std::tan(gamma/2)));
      t = 2*std::sqrt(-q)/std::sin(2*chi) - b/3;
    }
    else if (q == 0) {
      t = std::cbrt(2*r)-b/3;
    }else{
      gamma = std::atan(std::pow(q,3/2.)/r);
      chi = std::atan(std::cbrt(std::tan(gamma/2)));
      t = 2*std::sqrt(q)/std::tan(2*chi) - b/3; // cot(x) = tan(PI/2-x)
    }
  }else{ // three real solutions
    gamma = q==0? 0:std::acos(std::max(-1., std::min(r/std::pow(-q, 3/2.), 1.)));
    double min_dist = std::numeric_limits<double>::infinity(), dist;
    for (int i = 0; i < 3; i++) {
      chi = 2*std::sqrt(-q)*std::cos(gamma/3 + 2*i*PI/3) - b/3;
      if ((dist = dist2(qb2(p0,p1,p2,chi), m)) < min_dist) {
        min_dist = dist;
        t = chi;
      }
    }
  }
  return t ;
}

std::vector<std::array<double, 2>> Command_node::path_from_rrt(std::array<double, 2> goal, bool quadratic_bezier){
  // get the map
  ros::ServiceClient client = serviceClient<nav_msgs::GetMap>(enable_static_map_?"/static_map":"/dynamic_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid map;
  if (client.call(srv)){
    map=std::move(srv.response.map);
  }else{
    ROS_ERROR(enable_static_map_?"Failed to call service static_map":"Failed to call service dynamic_map");
    return std::vector<std::array<double, 2>>();
  }

  // get the current position
  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (ok()) {
    try{ // http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
      listener.lookupTransform("map","base_footprint", ros::Time(0), transform);
      break;
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }

  // return the path
  std::array<double, 2> start = {transform.getOrigin().x(), transform.getOrigin().y()}, origin={map.info.origin.position.x, map.info.origin.position.y};
  double res = map.info.resolution, dq=1, radius=0.5;
  std::vector<std::vector<int>> map_vec(map.info.height, std::vector<int>(map.info.width, 0));
  for (size_t i = 0; i < map.info.height; i++)
    for (size_t j = 0; j < map.info.width; j++)
      map_vec[i][j] = map.data[i*map.info.width+j];
  RRT rrt( map_vec, origin, res, dq, radius);

  if (quadratic_bezier){
    std::vector<std::array<double, 2>> path;
    while (path.size()<2) { // let the user to choose the path with opencv
      path =  rrt.safe_quadratic_bezier_path(rrt.connect(start, goal, ros::ok));
    }
    return path;
  }else{
    return rrt.connect(start, goal, ros::ok);
  }
}
