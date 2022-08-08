#ifndef HC_CC_CIRCLE_H
#define HC_CC_CIRCLE_H

#include <cassert>
#include <iostream>
#include <limits>

#include "configuration.h"
#include "utilities.h"

using namespace std;


class HC_CC_Circle_Param
{
public:
  /** \brief Set parameters */
  void set_param(double _kappa, double _sigma, double _radius, double _mu, double _sin_mu, double _cos_mu,
                double _delta_min);

  /** \brief Max. curvature, inverse of max. curvature, max. sharpness */
  double kappa, kappa_inv, sigma;

  /** \brief Radius of the outer circle */
  double radius;//@MSK: HC Circle or CC Circle radius at paper

  /** \brief Angle between the initial orientation and the tangent to the circle at the initial position */
  double mu;

  /** \brief Sine and cosine of mu */
  double sin_mu, cos_mu;

  /** \brief Minimal deflection */
  double delta_min;
};

class HC_CC_Circle : public HC_CC_Circle_Param
{
public:
  /** \brief Constructor */
  HC_CC_Circle(const Configuration &_start, bool _left, bool _forward, bool _regular, const HC_CC_Circle_Param &_param);

  /** \brief Constructor */
  HC_CC_Circle(double _xc, double _yc, bool _left, bool _forward, bool _regular, const HC_CC_Circle_Param &_param);

  /** \brief Computation of deflection (angle between start configuration of circle and configuration q) */
  double deflection(const Configuration &q) const;

  /** \brief Calculation of D1 for the evaluation of an elementary path */
  double D1(double alpha) const;

  /** \brief Computation of a rs-turn's circular deflection */
  double rs_circular_deflection(double delta) const;

  /** \brief Length of a rs-turn */
  double rs_turn_length(const Configuration &q) const;

  /** \brief Computation of a hc-turn's circular deflection */
  double hc_circular_deflection(double delta) const;

  /** \brief Length of a hc-turn */
  double hc_turn_length(const Configuration &q) const;

  /** \brief Computation of an elementary path's sharpness */
  bool cc_elementary_sharpness(const Configuration &q, double delta, double &sigma0) const;

  /** \brief Computation of a cc-turn's circular deflection */
  double cc_circular_deflection(double delta) const;

  /** \brief Length of a cc-turn */
  double cc_turn_length(const Configuration &q) const;

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Start configuration */
  Configuration start;

  /** \brief Turning direction: left/right */
  bool left;

  /** \brief Driving direction: forwards/backwards */
  bool forward;

  /** \brief Type of the circle: regular/irregular */ //MSK: regular turn or irregular turn
  bool regular;

  /** \brief Center of the circle */
  double xc, yc;
};

/** \brief Cartesian distance between the centers of two circles */
double center_distance(const HC_CC_Circle &c1, const HC_CC_Circle &c2);

/** \brief Configuration on the circle? */
bool configuration_on_hc_cc_circle(const HC_CC_Circle &c, const Configuration &q);

#endif
