#ifndef PATHS_H
#define PATHS_H

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
#include <numeric>

#include "configuration.h"
#include "hc_cc_circle.h"
#include "steering_functions.h"
#include "utilities.h"

using namespace std;

class Path
{
public:
  /** \brief Constructor */
  Path(const Configuration &_start, const Configuration &_end, double _kappa, double _sigma, double _length);

  /** \brief Start and end configuration */
  Configuration start, end;

  /** \brief Max. curvature (unsigned), max. sharpness (unsigned) */
  double kappa, sigma;

  /** \brief Path length */
  double length;
};

/** \brief cc-dubins path types: E (Empty), S (Straight), T (Turn) */
namespace cc_dubins
{
enum path_type
{
  E,
  S,
  T,
  TT,
  // Dubins families:
  TST,
  TTT,
  // #####################
  TTTT
};
}
const int nb_cc_dubins_paths = 7;

class CC_Dubins_Path : public Path
{
public:
  /** \brief Constructor */
  CC_Dubins_Path(const Configuration &_start, const Configuration &_end, cc_dubins::path_type _type, double _kappa,
                double _sigma, Configuration *_qi1, Configuration *_qi2, Configuration *_qi3, Configuration *_qi4,
                HC_CC_Circle *_cstart, HC_CC_Circle *_cend, HC_CC_Circle *_ci1, HC_CC_Circle *_ci2, double _length);

  /** \brief Destructor */
  ~CC_Dubins_Path();

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Path type */
  cc_dubins::path_type type;

  /** \brief Intermediate configurations */
  Configuration *qi1, *qi2, *qi3, *qi4;

  /** \brief Start, end and intermediate circles */
  HC_CC_Circle *cstart, *cend, *ci1, *ci2;
};

/** \brief hc-/cc-reeds-shepp path types: E (Empty), S (Straight), T (Turn), c (Cusp) */
namespace hc_cc_rs
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
const int nb_hc_cc_rs_paths = 18;

class HC_CC_RS_Path : public Path
{
public:
  /** \brief Constructor */
  HC_CC_RS_Path(const Configuration &_start, const Configuration &_end, hc_cc_rs::path_type _type, double _kappa,
                double _sigma, Configuration *_qi1, Configuration *_qi2, Configuration *_qi3, Configuration *_qi4,
                HC_CC_Circle *_cstart, HC_CC_Circle *_cend, HC_CC_Circle *_ci1, HC_CC_Circle *_ci2, double _length);

  /** \brief Destructor */
  ~HC_CC_RS_Path();

  /** \brief Alphanumeric display */
  void print(bool eol) const;

  /** \brief Path type */
  hc_cc_rs::path_type type;//@MSK: path 

  /** \brief Intermediate configurations */
  Configuration *qi1, *qi2, *qi3, *qi4;

  /** \brief Start, end and intermediate circles */
  HC_CC_Circle *cstart, *cend, *ci1, *ci2;
};

int direction(bool forward, bool order);

/** \brief Checks whether two hcStates are equal */
bool state_equal(const steer::hcState &state1, const steer::hcState &state2);

/** \brief Reverses a control */
void reverse_control(steer::Control &control);

/** \brief Subtracts control2 from control1 */
steer::Control subtract_control(const steer::Control &control1, const steer::Control &control2);

/** \brief Appends controls with 0 input */
void empty_controls(vector<steer::Control> &controls);

/** \brief Appends controls with a straight line */
void straight_controls(const Configuration &q1, const Configuration &q2, vector<steer::Control> &controls);

/** \brief Appends controls with a rs-turn */
void rs_turn_controls(const HC_CC_Circle &c, const Configuration &q, bool order, vector<steer::Control> &controls);

/** \brief Appends controls with a hc-turn */
void hc_turn_controls(const HC_CC_Circle &c, const Configuration &q, bool order, vector<steer::Control> &controls);

/** \brief Appends controls with an elementary path if one exists */
bool cc_elementary_controls(const HC_CC_Circle &c, const Configuration &q, double delta, bool order,
                            vector<steer::Control> &controls);

/** \brief Appends controls with a default cc-turn consisting of two clothoids and a circular arc */
void cc_default_controls(const HC_CC_Circle &c, const Configuration &q, double delta, bool order,
                        vector<steer::Control> &controls);

/** \brief Appends controls with a cc-turn */
void cc_turn_controls(const HC_CC_Circle &c, const Configuration &q, bool order, vector<steer::Control> &controls);


#endif
