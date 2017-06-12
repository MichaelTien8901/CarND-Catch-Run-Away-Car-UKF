#include "hunter.hpp"
#include <math.h>

#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

Hunter::Hunter() 
{
   state = -1;
   car_ = MatrixXd(3, 3); // x, y, timestamp
}

Hunter::~Hunter() 
{
}
void Hunter::FindHeading(const VectorXd &hunter, const VectorXd &x_, long long timestamp, double *turn_output)
{
	const int COLLECT_POINT_DELAY = 1000000;
	const double MAX_RADIUS = 20;
    double hunter_x, hunter_y, hunter_heading, p_x, p_y, target_x, target_y, velocity;
    hunter_x = hunter[0];
    hunter_y = hunter[1];
    hunter_heading = hunter(2); 

    switch(state) {
    case 5:
		if ( timestamp >= (timestamp_+100000)) { // expired, back to center
   	    	StoreCarCoordinate(x_, timestamp);
			timestamp_ = timestamp + COLLECT_POINT_DELAY;
			state = 3; // rest
			target_x = center_x;
			target_y = center_y; // try to back to center
			break;
		}
		target_x = intercept_x;
		target_y = intercept_y; 
		break;
     
    case 4:
   	    if ( timestamp >= timestamp_) {
   	 	// calculate target_x, target_y
   	    	StoreCarCoordinate(x_, timestamp);
			double px, py;
			velocity = x_[2];
			px = x_[0] - center_x;
			py = x_[1] - center_y;
			double px2py2 = px * px + py * py;
			double px2py2_sqrt = sqrtf(px2py2);
			float rho, phi, delta_phi;
			rho = px2py2_sqrt;
			radius = rho;
			double delta_t = radius / velocity;
			timestamp_ = timestamp + delta_t * 1000000;

			delta_phi = velocity / rho * delta_t;
			phi = atan2f(py, px) + delta_phi;

			intercept_x = rho * cos(phi) + center_x;
			intercept_y = rho * sin(phi) + center_y;
			cout << "intercept:" << intercept_x << "," << intercept_y << std::endl;

		   	target_x = intercept_x;
		   	target_y = intercept_y;
			state = 5;
			break;
	   	 }
	   	 target_x = center_x;
	   	 target_y = center_y;
	      // wait until meet at center
	   	 break;
    case 3:
		double a1, a2, a3, b1, b2, b3;
		cout << "point 0:" << car_(0, 0) <<"," << car_(0, 1) << std::endl; 
		cout << "point 1:" << car_(1, 0) <<"," << car_(1, 1) << std::endl; 
		cout << "point 2:" << car_(2, 0) <<"," << car_(2, 1) << std::endl; 

		CircleParameters(car_(0, 0), car_(0, 1), car_(1, 0), car_(1, 1), &a1, &a2, &a3);
		CircleParameters(car_(1, 0), car_(1, 1), car_(2, 0), car_(2, 1), &b1, &b2, &b3);

		if ( FindCircleCenter(a1, a2, a3, b1, b2, b3, &center_x, &center_y)) {
			cout << "center:" << center_x << "," << center_y << std::endl;
			target_x = center_x;
			target_y = center_y;
			radius = sqrt((car_(0, 1) - center_y)*(car_(0, 1) - center_y) + (car_(0, 0)- center_x)*(car_(0, 0)- center_x));
			cout << "radius:" << radius << std::endl;
			if ( radius < MAX_RADIUS) {
				double distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
				double velocity = x_[2];
				timestamp_ = timestamp + distance_difference / velocity * 1000000;
				state = 4;
			} else {
				timestamp_ = timestamp + COLLECT_POINT_DELAY;
		   		target_x = x_[0];
   			    target_y = x_[1];
   				state = 2;
			}
		} else {
      	 // shift 
			timestamp_ = timestamp + COLLECT_POINT_DELAY;
	   		target_x = x_[0];
   		    target_y = x_[1];
   			state = 2;
       }
       break;

    case -1: // initial
		timestamp_ = timestamp + COLLECT_POINT_DELAY;
    	state = 0;
    case 0:
    case 1:
    case 2:
   		if ( timestamp >= timestamp_) {
			StoreCarCoordinate(x_, timestamp);
			timestamp_ = timestamp + COLLECT_POINT_DELAY;
			state ++;
    	}
	default:
   		  target_x = x_[0];
   		  target_y = x_[1];
   		  break;
    }

	double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
	while (heading_to_target > M_PI) heading_to_target-=2.*M_PI; 
	while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
	//turn towards the target
	double heading_difference = heading_to_target - hunter_heading;
	while (heading_difference > M_PI) heading_difference-=2.*M_PI; 
	while (heading_difference <-M_PI) heading_difference+=2.*M_PI;
	*turn_output  = heading_difference;
}
void Hunter::CircleParameters( const double a, const double b, const double c, const double d, double *a1_out, double *a2_out, double *a3_out)
{
	*a1_out = 2 * (a - c);
	*a2_out = 2 * (b - d);
	*a3_out = c*c + d*d  - a*a - b*b;
	cout << "Circle Parameters- a1, a2, a3:" << *a1_out << ", "<< *a2_out << ", " << *a3_out << std::endl;
}
bool Hunter::FindCircleCenter( const double a1, const double b1, const double c1, const double a2, const double b2, const double c2, double *x_out, double *y_out )
{
	const double epsilon = 0.001;
	if ( fabs(a1*b2 - a2*b1) < epsilon ) {
		return false;
	}
	if ( (fabs(a1) < epsilon) && (fabs(b1) < epsilon) && (fabs(c1) < epsilon)) return false;
	if ( (fabs(a2) < epsilon) && (fabs(b2) < epsilon) && (fabs(c2) < epsilon)) return false;
	if ( (fabs(b1) < epsilon) && (fabs(b2) < epsilon)) return false;

	*x_out = (b2*c1-b1*c2) / (a2*b1-a1*b2);
	*y_out = (a2*c1-a1*c2) / (a1*b2-a2-b1);
	cout << "FindCircleCenter:" << *x_out << ", " <<  *y_out << std::endl;
	return true;
}
void Hunter::StoreCarCoordinate(const VectorXd &x_, const long long timestamp)
{
	for ( int i = 0; i < 2; i ++) {
      	car_(i, 0) = car_(i+1, 0);
	  	car_(i, 1) = car_(i+1, 1);
	  	car_(i, 2) = car_(i+1, 2);
	}
	car_(2, 0) = x_[0];
	car_(2, 1) = x_[1];
	car_(2, 2) = timestamp;
}