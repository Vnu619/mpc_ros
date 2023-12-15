#ifndef MPC_H
#define MPC_H

#include <vector>
#include "../src/Eigen-3.3/Eigen/Core"
//#include <pcl/filters/passthrough.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>
//

using namespace std;

struct Point2D {
    float x;
    float y;
};
struct Point {
    double x;
    double y;
    double yaw;
};
class MPC {

 double cte_weight_;
 double epsi_weight_;
 double v_weight_;
 double delta_weight_;
 double a_weight_;
 double delta_gap_weight_;
 double a_gap_weight_;
 double ref_velocity_;

 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, std::vector<Point2D> obstacles);

  void init(double cte_weight, double epsi_weight, double v_weight, double delta_weight,
            double a_weight, double delta_gap_weight, double a_gap_weight,
            double ref_velocity);
};

#endif /* MPC_H */
