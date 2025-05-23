#ifndef UTILITIES_H
#define UTILITIES_H

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>


#include "fresnel.data"

using namespace std;

#define PI 3.1415926535897932384
#define HALF_PI 1.5707963267948966192
#define TWO_PI 6.2831853071795864770
#define SQRT_PI 1.7724538509055160273
#define SQRT_PI_INV 0.56418958354775628695
#define SQRT_TWO_PI_INV 0.39894228040143267794

const double epsilon = 1e-4;

/** \brief Return value of epsilon */
double get_epsilon();

/** \brief Return sign of a number */
double sgn(double x);

/** \brief Cartesian distance between two points */
double point_distance(double x1, double y1, double x2, double y2);

/** \brief Computation of a point's polar coordinates */
void polar(double x, double y, double &r, double &theta);

/** \brief Conversion of arbitrary angle given in [rad] to [0, 2*pi[ */
double twopify(double alpha);

/** \brief Conversion of arbitrary angle given in [rad] to [-pi, pi[ */
double pify(double alpha);

/** \brief Fresnel integrals: S_f = int_0_s(sin(pi/2 u*u)du), C_f = int_0_s(cos(pi/2 u*u)du) approximated with Chebyshev
    polynomials
    */
void fresnel(double s, double &S_f, double &C_f);

/** \brief Computation of the end point on a clothoid
    x_i, y_i, theta_i, kappa_i: initial configuration
    sigma: sharpness of clothoid
    direction: driving direction {-1.0, 1.0}
    length: length of clothoid (positive)
    x_f, y_f, theta_f, kappa_f: final configuration on clothoid
    */
void end_of_clothoid(double x_i, double y_i, double theta_i, double kappa_i, double sigma, double direction,
                    double length, double *x_f, double *y_f, double *theta_f, double *kappa_f);

/** \brief Computation of the end point on a circular arc
    x_i, y_i, theta_i: initial configuration
    kappa: curvature of circular arc
    direction: driving direction {-1.0, 1.0}
    length: arc length (positive)
    x_f, y_f, theta_f: final configuration on circular arc
    */
void end_of_circular_arc(double x_i, double y_i, double theta_i, double kappa, double direction, double length,
                        double *x_f, double *y_f, double *theta_f);

/** \brief Computation of the end point on a straight line
    x_i, y_i: initial configuration
    theta: angle of straight line
    direction: driving direction {-1.0, 1.0}
    length: line length (positive)
    x_f, y_f: final configuration on straight line
    */
void end_of_straight_line(double x_i, double y_i, double theta, double direction, double length, double *x_f,
                        double *y_f);

/** \brief Transformation of (local_x, local_y) from local coordinate system to global one */
void global_frame_change(double x, double y, double theta, double local_x, double local_y, double *global_x,
                        double *global_y);

/** \brief Transformation of (global_x, global_y) from global coordinate system to local one */
void local_frame_change(double x, double y, double theta, double global_x, double global_y, double *local_x,
                        double *local_y);

/** \brief Find index with minimal value in double array */
int array_index_min(double array[], int size);

/** \brief Initialize an array with a given value */
void double_array_init(double array[], int size, double value);

/** \brief Initialize an array with nullptr */
void pointer_array_init(void *array[], int size);

vector<double> linspace(double a, double b, int num);
vector<double> linspace(double a, double b, int num, bool include_end);

#endif
