#ifndef GC_PATHS_H
#define GC_PATHS_H

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
#include <numeric>

#include "configuration.h"
#include "paths.h"
#include "gc_circle.h"
#include "steering_functions.h"
#include "utilities.h"
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>


using namespace std;
using namespace steer;

class Path_for_gc
{
public:
  /** \brief Constructor */
  Path_for_gc(const Configuration &_start, const Configuration &_end, double _kappa, double _sigma_sc, double _sigma_cs, double _length);

  /** \brief Start and end configuration */
  Configuration start, end;

  /** \brief Max. curvature (unsigned), max. sharpness (unsigned) */
  double kappa, sigma_sc, sigma_cs;

  /** \brief Path length */
  double length;
};

/** \brief hc-/cc-reeds-shepp path types: E (Empty), S (Straight), T (Turn), c (Cusp) */
namespace hc_gc_rs
{
enum path_type
{
  E,
  S,
  T,
  TT,
  TcT,
  // Reeds-Shepp families:
  TcTcT,
  TcTT,
  TTcT,
  TST,
  TSTcT,
  TcTST,
  TcTSTcT,
  TTcTT,
  TcTTcT,
  // ##################### @MSK: paper에서의 정의한 새로운 HC family
  TTT,
  TcST,
  TScT,
  TcScT
};
}
const int nb_hc_gc_rs_paths = 18;

class GC_Path : public Path_for_gc
{
public:
  /** \brief Constructor */
  GC_Path(const Configuration &_start, const Configuration &_end, hc_gc_rs::path_type _type, double _kappa,
                double _sigma_sc, double _sigma_cs, Configuration *_qi1, Configuration *_qi2, Configuration *_qi3, Configuration *_qi4,
                GC_Circle *_cstart, GC_Circle *_cend, GC_Circle *_ci1, GC_Circle *_ci2, double _length);

  /** \brief Destructor */
  ~GC_Path();

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Path type */
  hc_gc_rs::path_type type;//@MSK: path 

  /** \brief Intermediate configurations */
  Configuration *qi1, *qi2, *qi3, *qi4;

  /** \brief Start, end and intermediate circles */
  GC_Circle *cstart, *cend, *ci1, *ci2;
};

/** \brief Appends controls with a rs-turn */
void gc_rs_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

/** \brief Appends controls with a hc-turn */
void gc_hc_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

/** \brief Appends controls with an elementary path if one exists */
bool gc_elementary_controls(const GC_Circle &c, const Configuration &q, double delta, bool order,
                            bool straight_to_circle, vector<Control> &controls);

/** \brief Appends controls with a default cc-turn consisting of two clothoids and a circular arc */
void gc_default_controls(const GC_Circle &c, const Configuration &q, double delta, bool order,
                         vector<Control> &controls);

/** \brief Appends controls with a cc-turn */
void gc_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

#endif
