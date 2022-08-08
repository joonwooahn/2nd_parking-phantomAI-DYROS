#ifndef GC_CIRCLE_H
#define GC_CIRCLE_H

#include <cassert>
#include <iostream>
#include <limits>

#include "configuration.h"
#include "utilities.h"

using namespace std;

class GC_Circle_Param
{
public:
  /** \brief Set parameters */
  void set_param(double _kappa, double _sigma_sc, double _sigma_cs, double _radius_sc, 
                 double _mu, double _sin_mu, double _cos_mu, double _gamma,
                 double _radius_cs, double _mu2, double _sin_mu2, double _cos_mu2, 
                 double _delta_min);

  /** \brief Max. curvature, inverse of max. curvature, max. sharpness */
  double kappa, kappa_inv, sigma_cs;

  double gamma;// velocity 비율
  double velo_min;
  double velo_ref;

  /** \brief Radius of the outer circle */
  /** \brief Angle between the initial orientation and the tangent to the circle at the initial position */
  double sigma_sc;
  double radius_sc;//@MSK: HC Circle or CC Circle radius at paper
  double mu_sc;
  double sin_mu_sc, cos_mu_sc;

  /** \brief 2 is activated when backwards moving.... */
  double radius_cs;//
  double mu_cs;
  double sin_mu_cs, cos_mu_cs;
  
  /** \brief if you use elementary, the path is violated the velocity control profile... such that default settting is 'false' */
  bool using_elementary = true;

  /** \brief Minimal deflection */
  double delta_min;
};

class GC_Circle : public GC_Circle_Param
{
public:
  /** \brief Constructor */
  GC_Circle(const Configuration &_start, bool _left, bool _forward, bool _regular, bool _straight_to_circle, const GC_Circle_Param &_param);

  /** \brief Constructor */
  GC_Circle(double _xc, double _yc, bool _left, bool _forward, bool _regular, const GC_Circle_Param &_param);

  /** \brief Computation of deflection (angle between start configuration of circle and configuration q) */
  double deflection(const Configuration &q) const;

  /** \brief Computation of a rs-turn's circular deflection */
  double rs_circular_deflection(double delta) const;

  /** \brief Length of a rs-turn */
  double rs_turn_length(const Configuration &q) const;

  // /** \brief Computation of a hc-turn's circular deflection */
  // double hc_circular_deflection(double delta) const;
  double D1(double alpha) const;

  /** \brief Computation of a gc-turn's circular deflection */
  double hc_circular_deflection(double delta, double delta_min) const;

  /** \brief Length of a hc-turn */
  double hc_turn_length(const Configuration &q, bool forward) const;

  /** \brief Computation of an elementary path's sharpness */
  bool gc_elementary_sharpness(const Configuration &q, double delta, double &sub_straight, double &sigma0, double &kappa0, bool straight_to_circle) const;

  /** \brief Computation of a gc-turn's circular deflection */
  double gc_circular_deflection(double delta) const;

  /** \brief Length of a gc-turn */
  double gc_turn_length(const Configuration &q, bool straight_to_circle) const;

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Start configuration */
  Configuration start;

  /** \brief Turning direction: left/right */
  bool left;

  /** \brief Driving direction: forwards/backwards */
  bool forward;

  /** \brief Type of the circle: regular/irregular */ //MSK: regular turn or irregular turn (Elementary path?)
  bool regular;

  /** \brief Center of the circle. 만약 현재 configuration의 방향성이 직진 -> 클로소이드 -> 원 */
  double xc, yc;
  
  // /** \brief Center of the circle. 만약 현재 configuration의 방향성이 원 -> 클로소이드 -> 직선, i.e backward의 경우...*/
  // double xc_b, yc_b;
};

/** \brief Cartesian distance between the centers of two circles */
double center_distance(const GC_Circle &c1, const GC_Circle &c2);

/** \brief Configuration on the circle? */
bool configuration_on_gc_circle(const GC_Circle &c, const Configuration &q, bool straight_to_circle);

#endif
