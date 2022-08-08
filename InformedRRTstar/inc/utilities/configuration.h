#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>

#include "utilities.h"

using namespace std;

class Configuration
{
public:
  /** \brief Constructor */
  Configuration(double _x = 0.0, double _y = 0.0, double _theta = 0.0, double _kappa = 0.0);

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Position */
  double x, y;

  /** \brief Orientation in rad between [0, 2*pi[ */
  double theta;

  /** \brief Curvature */
  double kappa;
};


/** \brief Cartesian distance between two configurations */
double configuration_distance(const Configuration &q1, const Configuration &q2);

/** \brief Are two configurations aligned? */
bool configuration_aligned(const Configuration &q1, const Configuration &q2);

/** \brief Are two configurations equal? */
bool configuration_equal(const Configuration &q1, const Configuration &q2);

#endif
