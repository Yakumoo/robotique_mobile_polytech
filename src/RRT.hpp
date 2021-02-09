#ifndef RRT_HPP
#define RRT_HPP

#include "phtree-cpp/phtree/phtree.h"
#include "phtree-cpp/phtree/phtree_d.h"
#include <vector>
#include <random>
#include <memory>

#ifdef OPENCV
#include <opencv2/core.hpp>
#endif

using namespace improbable::phtree;


class RRT{
  private:
    PhTreeD<2, PhPointD<2>> treeS_, treeG_; // start and goal trees
    PhTree<2, bool> tree_map_; // the value is useless
    PhPointD<2> lastS_, lastG_, origin_, qnew_, qnear_; // store the last added node
    double delta_, radius_, res_, dq_;
    int rows_, columns_;
    std::vector<std::vector<int>> map_;
    std::default_random_engine gen_; // random generator
    std::uniform_real_distribution<double> urdx_, urdy_;
    bool try_;
    PhDistanceDoubleEuclidean<2> distD_;
    PhDistanceLongEuclidean<2> dist_;

    #ifdef OPENCV
      cv::Mat image_;
      // click on the image to set the goal position otherwise the value is untouched, then press a key to let the tree grow
      cv::Mat cvMap(PhPointD<2> start, PhPointD<2>& goal);
      // highlight of the path
      void show_path(const std::vector<PhPointD<2>> & path);
    #endif

    // return a new point between qnear and qrand with a distance dq_ from qnear
    PhPointD<2> new_conf(PhPointD<2> qnear,PhPointD<2> qrand);
    bool connection(PhPointD<2> node, PhPointD<2> goal);
    bool collision(PhPointD<2> p);
    bool connection_tree(PhTreeD<2,PhPointD<2>>& tree, PhPointD<2> query, PhPointD<2>& qnew_store_goal);
    // return list of points with backward shortcut including the goal point which has to be in the tree and not including the start point
    std::vector<PhPointD<2>> backward(PhTreeD<2,PhPointD<2>>& tree, PhPointD<2> goal);
    // return list of points with forward shortcut including the start point at the beginning and the goal point which is the last element of "path"
    std::vector<PhPointD<2>> forward(std::vector<PhPointD<2>> path, PhPointD<2> start);

    double length_quadratic_bezier(PhPointD<2> start, PhPointD<2> control, PhPointD<2> end);
    bool connection_quadratic_bezier(PhPointD<2> start, PhPointD<2> control, PhPointD<2> end);
    // solve the linear system equation using the tridiagonal matrix algorithm
    std::vector<PhPointD<2>> solve_quadratic_bezier_path(std::vector<PhPointD<2>> path);
    bool collision_path_quadratic_bezier(std::vector<PhPointD<2>> path);

  public:
    RRT( std::vector<std::vector<int>> map, PhPointD<2> origin, double res, double dq=0.5, double radius=0.3);
    virtual ~RRT();
    // build a rrt and show the graph with to openCV if enable
    std::vector<PhPointD<2>> build(PhPointD<2> start, PhPointD<2> goal, bool(*cond)()=[](){return true;});
    // rrt connect: build 2 trees (start and goal tree)
    std::vector<PhPointD<2>> connect(PhPointD<2> start, PhPointD<2> goal, bool(*cond)()=[](){return true;});
    // build a set of quadratic bezier curves without any collision given by the list of point "path"
    std::vector<PhPointD<2>> safe_quadratic_bezier_path(std::vector<PhPointD<2>> path);
};

#endif
