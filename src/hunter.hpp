#ifndef HUNTER_H_
#define HUNTER_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Hunter {
public:
  /**
  * Constructor.
  */
  Hunter();

  /**
  * Destructor.
  */
  virtual ~Hunter();

  /**
  * A helper method to calculate RMSE.
  */
  void FindHeading(const VectorXd &hunter, const VectorXd &x_, long long timestamp, double *turn_output);
  void CircleParameters( const double a, const double b, const double c, const double d, double *a1_out, double *a2_out, double *a3_out);
  bool FindCircleCenter( const double a1, const double b1, const double c1, const double a2, const double b2, const double c2, double *x_out, double *y_out );
private:
  void StoreCarCoordinate(const VectorXd &x_, const long long timestamp);
  int state;
  MatrixXd car_;
  double center_x, center_y, radius, intercept_x, intercept_y;
  long long timestamp_;

};

#endif /* HUNTER_H_ */
