#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#include <iostream>

#include "RRT.hpp"

#define CONT2DISC(p) static_cast<int>(((p)[0]-origin_[0])/res_),static_cast<int>(((p)[1]-origin_[1])/res_)
#define RANDCONF {urdx_(gen_),urdy_(gen_)}
#define QB(s,c,e,t) {std::pow(1-(t),2)*(s)[0] + 2*(1-(t))*(t)*(c)[0] + std::pow(t,2)*(e)[0],\
                     std::pow(1-(t),2)*(s)[1] + 2*(1-(t))*(t)*(c)[1] + std::pow(t,2)*(e)[1]}
// do not use it!
#define _TRYINSERT(S,G,query) \
  qnear_ = tree##S##_.begin_knn_query(1, query).first(); \
  qnew_ = new_conf(qnear_, query);\
  if (connection(qnear_, qnew_) && !collision(qnew_)) { \
    last##S##_ = qnew_;\
    tree##S##_.insert(qnew_, qnear_);\
    try_=false;\
    if (connection_tree(tree##G##_, qnew_, last##G##_)) {\
      tree##S##_.insert(last##G##_, qnew_);\
      goto pathFound;\
    }

#ifdef OPENCV
  #include <opencv2/core.hpp>
  #include <opencv2/highgui.hpp>
  #include <opencv2/imgproc.hpp>
  #include <opencv2/viz/types.hpp>
  #define TRYINSERT(S,G,query) _TRYINSERT(S,G,query)\
    cv::line(image_, cv::Point(CONT2DISC(qnew_)), cv::Point(CONT2DISC(qnear_)), cv::Scalar(175), 1);}
#else
  #define TRYINSERT(S,G,query) _TRYINSERT(S,G,query)}
#endif
//using namespace std::placeholders;

RRT::RRT( std::vector<std::vector<int>> map, PhPointD<2> origin, double res, double dq, double radius):
    urdx_(origin[0], origin[0]+map[0].size()*res), urdy_(origin[1], origin[1]+map.size()*res), tree_map_(), treeS_(), treeG_(),
    map_(map), origin_(origin), res_(res), dq_(dq), rows_(map.size()), columns_(map[0].size()), radius_(radius){

  std::random_device r; //https://en.cppreference.com/w/cpp/numeric/random
  std::default_random_engine e1(r());
  gen_=e1;
  // the key is the node, the value is the previous node
  // the previous node of the start node is itself
  int ip,im,jp,jm;
  for (int i=0; i<rows_; i++){ // y
    ip = std::min(rows_,i+1);
    im = std::max(0,i-1);
    for (int j=0; j<columns_; j++) { // x
      jp = std::min(columns_,j+1);
      jm = std::max(0,j-1);
      if (map_[i][j]>50 && !(map_[ip][j]>50 && map_[i][jp]>50 && map_[im][j]>50 && map_[i][jm]>50)) {
        PhPoint<2> wall = {j,i}; // if it is a contour
        tree_map_.insert(wall,1);
      }
    }
  }
#ifdef OPENCV
  cv::namedWindow("rrt", cv::WINDOW_NORMAL);
  cv::resizeWindow("rrt", 1088, 592);
#endif
}

RRT::~RRT(){
#ifdef OPENCV
  cv::destroyWindow("rrt");
#endif
}


std::vector<PhPointD<2>> RRT::build(PhPointD<2> start, PhPointD<2> goal, bool(*cond)()){
  // show the map
  #ifdef OPENCV
    image_ = cvMap(start, goal);
  #endif
  // init
  treeS_.clear(); treeS_.insert(start,start); lastS_=start;
  treeG_.clear(); treeG_.insert(goal , goal); lastG_=goal;
  // check
  if (collision(start) || collision(goal) || connection(start, goal))
    return std::vector<PhPointD<2>>{start, goal}; // empty
  // start loop
  PhPointD<2> r=RANDCONF;
  while (cond()){
    for(try_=true; try_; r=RANDCONF) {TRYINSERT(S,G,r)}
    #ifdef OPENCV
      cv::imshow("rrt", image_);
      if (cv::waitKey(1) ==27) return std::vector<PhPointD<2>>();
    #endif
  }pathFound:// goal found
  std::vector<PhPointD<2>> path = forward(backward(treeS_, goal), start);

  #ifdef OPENCV
    show_path(path);
  #endif

  return path;
}

std::vector<PhPointD<2>> RRT::connect(PhPointD<2> start, PhPointD<2> goal, bool(*cond)()){
  #ifdef OPENCV
    image_ = cvMap(start, goal);
  #endif

  treeS_.clear(); treeS_.insert(start,start); lastS_=start;
  treeG_.clear(); treeG_.insert(goal , goal); lastG_=goal;

  if (collision(start) || collision(goal) || connection(start, goal))
    return std::vector<PhPointD<2>>{start, goal}; // empty

  PhPointD<2> r=RANDCONF;
  while (cond()){
    for(try_=true; try_; r=RANDCONF) {TRYINSERT(S,G,r)}
    TRYINSERT(G,S,lastS_)
    for(try_=true; try_; r=RANDCONF) {TRYINSERT(G,S,r)}
    TRYINSERT(S,G,lastG_)
    #ifdef OPENCV
      cv::imshow("rrt", image_);
      if (cv::waitKey(1) == 27) return std::vector<PhPointD<2>>();
    #endif
  }pathFound:

  std::vector<PhPointD<2>> path_start = backward(treeS_,lastS_), path_goal = backward(treeG_,lastG_);
  for (std::vector<PhPointD<2>>::reverse_iterator it=path_goal.rbegin(); it!=path_goal.rend(); it++)
    path_start.push_back(*it); // concat both paths
  path_start.push_back(goal);
  path_start = forward(path_start, start);

  #ifdef OPENCV
    show_path(path_start);
  #endif

  return path_start;
}


PhPointD<2> RRT::new_conf(PhPointD<2> qnear,PhPointD<2> qrand){
  double norm = distD_(qnear, qrand);
  return PhPointD<2> {qnear[0]+(qrand[0]-qnear[0])/norm*dq_, qnear[1]+(qrand[1]-qnear[1])/norm*dq_};
}

bool RRT::connection(PhPointD<2> node,PhPointD<2> goal){
  double a, b, dx=goal[0]-node[0], dy=goal[1]-node[1], splitSize;
  int sign, end, split=1, minNode, i;
  PhPoint<2> test;
  a = static_cast<double>(dy)/dx; // line equation
  b = (goal[1]-origin_[1] - a*(goal[0]-origin_[0])) / res_;
  if (std::abs(dx) >= std::abs(dy)) {
    splitSize = std::abs(dx)/res_;
    minNode = (std::min(node[0], goal[0])-origin_[0]) / res_;
    while (splitSize > 2) { // dichotomy
      for (i = 0; i < split; i++) {
        test[0] = minNode + splitSize/2 + i*splitSize;
        test[1] = a*test[0] + b; // y=a*x+b
        if (dist_(test, tree_map_.begin_knn_query(1, test).first()) < radius_/res_) return false;
      }
      split *= 2;
      splitSize /= 2;
    }
  }else{
    splitSize = std::abs(dy)/res_;
    minNode = (std::min(node[1], goal[1])-origin_[1]) / res_ ;
    while (splitSize > 2) {
      for (i = 0; i < split; i++) {
        test[1] = minNode + splitSize/2 + i*splitSize;
        test[0] = (test[1]-b)/a; // x=(y-b)/a
        if (dist_(test, tree_map_.begin_knn_query(1, test).first()) < radius_/res_) return false;
      }
      split *= 2;
      splitSize /= 2;
    }
  }
  return true;
}

bool RRT::collision(PhPointD<2> p){
  PhPoint<2> discrete = {CONT2DISC(p)};
  return dist_(discrete, tree_map_.begin_knn_query(1, discrete).first()) < radius_/res_;
}


bool RRT::connection_tree(PhTreeD<2,PhPointD<2>>& tree, PhPointD<2> node, PhPointD<2>& lastG){
  // check 2-nearest node in the tree
  for (auto it = tree.begin_knn_query(2, node); it != tree.end(); ++it){
    if (connection(node, it.first())){
      lastG = it.first();
      return true;
    }
  }
  return connection(node, lastG); // finally try the last inserted
}

std::vector<PhPointD<2>> RRT::backward(PhTreeD<2,PhPointD<2>>& tree, PhPointD<2> goal){
  std::vector<PhPointD<2>> path = {goal}; // from goal to start
  PhPointD<2> qlast=goal, qnode, qprev=goal;
  do { // backward process
    qnode = qprev;
    qprev = tree.find(qnode).second();
    #ifdef OPENCV
      cv::line(image_, cv::Point(CONT2DISC(qprev)), cv::Point(CONT2DISC(qnode)), cv::Scalar(175), 2);
    #endif
    if (!connection(qprev, qlast)) { // try to find the furthest node
      path.insert(path.begin(),qnode);
      qlast = qnode;
    }
  } while (qprev != qnode);
  return path;
}

std::vector<PhPointD<2>> RRT::forward(std::vector<PhPointD<2>> path, PhPointD<2> start){
  std::vector<PhPointD<2>> filtered = {start};
  PhPointD<2> current = start, last = path[0];
  for (const PhPointD<2> & point : path){
    if (!connection(current, point)) {
      filtered.push_back(last);
      current = last;
    }
    last = point;
  }
  filtered.push_back(*--path.end());
  return filtered;
}

std::vector<PhPointD<2>> RRT::solve_quadratic_bezier_path(std::vector<PhPointD<2>> path){
  //https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
  // a=1/6, b=1, c=1/6, d=2*(path[i]+path[i+1])/3, w=1/6
  if (path.size() < 3) {
    return path;
  }else if (path.size() == 3){
    return std::vector<PhPointD<2>> {path[0],
    {2*path[1][0] - (path[0][0]+path[2][0])/2, 2*path[1][1] - (path[0][1]+path[2][1])/2},
    path[2]};
  }else if (path.size() == 4) {
    PhPointD<2> m = {(2*(path[1][0]+path[2][0]) - (path[0][0]+path[3][0])/2) / 3, (2*(path[1][1]+path[2][1]) - (path[0][1]+path[3][1])/2) / 3};
    return std::vector<PhPointD<2>> {path[0],
      {2*path[1][0]-(path[0][0]+m[0])/2, 2*path[1][0]-(path[0][0]+m[0])/2},
      m,
      {2*path[2][0]-(path[3][0]+m[0])/2, 2*path[2][0]-(path[3][0]+m[0])/2},
      path[3]};
  }
  // init
  int i, l=2*path.size()-3;
  double b = 35/36.;
  std::vector<PhPointD<2>> d(path.size()-3), new_path(l);
  PhPointD<2> previous = path[0];
  new_path[0] = path[0];
  new_path[l-1] = path[path.size()-1]; // add last
  // forward sweep
  for (i = 0; i < d.size()-1; i++) {
    d[i][0] = 2*(path[i+1][0]+path[i+2][0])/3 - previous[0]/6;
    d[i][1] = 2*(path[i+1][1]+path[i+2][1])/3 - previous[1]/6;
    previous = d[i];
  }
  d[i][0] = 2*(path[i+1][0]+path[i+2][0])/3 - path[i+3][0]/6 - previous[0]/6;
  d[i][1] = 2*(path[i+1][1]+path[i+2][1])/3 - path[i+3][1]/6 - previous[1]/6;
  // back substitution
  previous = {0,0};
  for (i = d.size()-1; i >= 0 ; i--) {
    new_path[2*i+2][0] = (d[i][0] - previous[0]/6) / b; // add new middle point
    new_path[2*i+2][1] = (d[i][1] - previous[1]/6) / b;
    new_path[2*i+3][0] = 2*path[i+2][0] - (new_path[2*i+2][0]+new_path[2*(i+2)][0])/2; // update the point to pass through
    new_path[2*i+3][1] = 2*path[i+2][1] - (new_path[2*i+2][1]+new_path[2*(i+2)][1])/2;
    previous = new_path[2*i+2];
  }
  new_path[1][0] = 2*path[1][0] - (path[0][0]+new_path[2][0])/2; // add second
  new_path[1][1] = 2*path[1][1] - (path[0][1]+new_path[2][1])/2;
  return new_path;
}

std::vector<PhPointD<2>> RRT::safe_quadratic_bezier_path(std::vector<PhPointD<2>> path){
  // TODO: optimization: reduce points with mean, crossover, both ...
  if (path.size()==0) return path;
  double max_dist, dist;
  int i;
  std::vector<PhPointD<2>>::iterator max_it;
  std::vector<PhPointD<2>> new_path = solve_quadratic_bezier_path(path);
  for (i = 1; i < path.size()-1; i++){
    if (!connection_quadratic_bezier(new_path[2*i-2], new_path[2*i-1], new_path[2*i])){
      max_dist=0;
      for (int j = std::max(0,i-2); j < std::min<int>(path.size()-1,i+2); j++) { // try neighbors
        if ((dist = distD_(path[j], path[j+1])) > max_dist) {
          max_dist = dist;
          max_it = path.begin()+j+1;
        }
      }
      // add middle point and update
      path.insert(max_it, PhPointD<2>{((*std::prev(max_it))[0]+(*max_it)[0])/2, ((*std::prev(max_it))[1]+(*max_it)[1])/2});
      new_path = solve_quadratic_bezier_path(path);
      i=1; // restart
    }
  }


  #ifdef OPENCV
    if(image_.empty()) image_ = cvMap(path[0], path[path.size()-1]);
    for (const auto & e : new_path){
      cv::circle(image_, cv::Point(CONT2DISC(e)), 1, cv::Scalar(120), 20);
    }
    double bezier_length;
    int step_res = 4; // the lower, the better the quality is
    PhPointD<2> last, current;
    for (int i = 1; i < new_path.size()-1; i+=2) {
      bezier_length = length_quadratic_bezier(new_path[i-1], new_path[i], new_path[i+1])/res_/step_res;
      last = QB(new_path[i-1], new_path[i], new_path[i+1], 0);
      for (int j = 1; j < bezier_length; j++) {
        current = QB(new_path[i-1], new_path[i], new_path[i+1], j/bezier_length);
        cv::line(image_, cv::Point(CONT2DISC(current)), cv::Point(CONT2DISC(last)), cv::Scalar(120), 5);
        last = current;
      }
    }
    cv::resizeWindow("rrt", 1088, 592);
    cv::imshow("rrt", 255-image_);
    if (cv::waitKey(0)==27) return std::vector<PhPointD<2>>();
  #endif
  return new_path;
}

bool RRT::collision_path_quadratic_bezier(std::vector<PhPointD<2>> path){
  for (int i = 1; i < path.size(); i+=2)
    if (!connection_quadratic_bezier(path[i-1], path[i], path[i+1]))
      return true;
  return false;
}

bool RRT::connection_quadratic_bezier(PhPointD<2> start, PhPointD<2> control, PhPointD<2> end){
  int split=1, i;
  double t=1, tinter;
  PhPoint<2> test;
  PhPointD<2> testD;
  for (double splitSize = length_quadratic_bezier(start, control, end)/res_ ; splitSize>0.5; splitSize/=2) {
    for (i = 0; i < split; i++) {
      tinter = t/2 + t*i;
      testD = QB(start, control, end, tinter);
      test = PhPoint<2>{CONT2DISC(testD)};
      if (dist_(test, tree_map_.begin_knn_query(1, test).first()) < radius_/res_) return false;
    }
    split*=2;
    t/=2;
  }
  return true;
}

double RRT::length_quadratic_bezier(PhPointD<2> start, PhPointD<2> control, PhPointD<2> end){
  // there is maybe a scale factor error
  // https://stackoverflow.com/a/11857788
  PhPointD<2>
    pa={start[0]-2*control[0] + end[0], start[1]-2*control[1] + end[1]},
    pb={2*(control[0]-start[0]), 2*(control[1]-start[1])};
  double t=1, A=4*(pa[0]*pa[0]+pa[1]*pa[1]), B=4*(pa[0]*pb[0]+pa[1]*pb[1]), C=pb[0]*pb[0]+pb[1]*pb[1], b=B/(2*A), u=t+b, k=C/A-b*b,
    sqrtu=std::sqrt(u*u+k), sqrtb=std::sqrt(b*b+k);
  return std::sqrt(A)/2 * (u*sqrtu-b*sqrtb+k*std::log(std::abs((u+sqrtu)/(b+sqrtb))));
}



#ifdef OPENCV
cv::Mat RRT::cvMap(PhPointD<2> start, PhPointD<2>& goal){
  PhPoint<2> goal_disc = {CONT2DISC(goal)};
  cv::setMouseCallback("rrt", [](int event, int x, int y, int flags, void* param){*(PhPoint<2>*)param = {x,y};}, (void*)&goal_disc);
  cv::Mat image_ = cv::Mat::zeros(rows_, columns_, CV_8UC1);
  unsigned char* p;
  for (int i=0; i<rows_; i++){ // y
    p=image_.ptr<unsigned char>(i);
    for (int j=0; j<columns_; j++){
      switch (map_[i][j]) {
        case -1:  p[j]=100; break; // unknown
        case 0:   p[j]=0;   break; // free
        case 100: p[j]=100; break; // wall
      }
    }
  }
  cv::circle(image_, cv::Point(CONT2DISC(start)), 1, cv::Scalar(175), 30);
  cv::imshow("rrt", 255-image_);
  cv::waitKey(0);
  cv::setMouseCallback("rrt", NULL, NULL);
  goal = {goal_disc[0]*res_+origin_[0], goal_disc[1]*res_+origin_[1]}; // set the new goal
  cv::circle(image_, cv::Point(CONT2DISC(goal)), 1, cv::Scalar(175), 30);
  return image_;
}

void RRT::show_path(const std::vector<PhPointD<2>> & path){
  for (auto it = path.begin() ; it!=--path.end(); it++) {
    cv::circle(image_, cv::Point(CONT2DISC(*std::next(it))), 1, cv::Scalar(120), 20);
    cv::line(image_, cv::Point(CONT2DISC(*std::next(it))), cv::Point(CONT2DISC(*it)), cv::Scalar(200), 4);
  }
  cv::resizeWindow("rrt", 1088, 592);
  cv::imshow("rrt", 255-image_);
  cv::waitKey(0);
}
#endif
