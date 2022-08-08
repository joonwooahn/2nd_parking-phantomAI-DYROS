#ifndef GC_STATE_SPACE_H
#define GC_STATE_SPACE_H

#include <cmath>
#include <vector>
#include <iomanip>
#include "ekf.h"
#include "gc_circle.h"
#include "steering_functions.h"
#include "utilities.h"

using namespace std;
using namespace steer;

class GC_State_Space
{
public:
  /** \brief Constructor */
  GC_State_Space(double kappa, double sigma_cs, double gamma, double discretization);

  /** \brief Sets the parameters required by the filter */
  void set_filter_parameters(const Motion_Noise& motion_noise, const Measurement_Noise& measurement_noise,
                             const Controller& controller);

  /** \brief Virtual function that returns controls of the shortest path from state1 to state2 */
  virtual vector<Control> get_controls(const hcState& state1, const hcState& state2) const = 0;

  // virtual vector<Configuration> get_configurations();

  /** \brief Returns path from state1 to state2 */
  vector<hcState> get_path(const hcState& state1, const hcState& state2) const;
  // pair<vector<State>, vector<Configuration>> get_path(const State& state1, const State& state2) const;

  /** \brief Returns path including covariances from state1 to state2 */
  vector<hcState_With_Covariance> get_path_with_covariance(const hcState_With_Covariance& state1,
                                                         const hcState& state2) const;

  /** \brief Returns integrated states given a start state and controls */
  vector<hcState> integrate(const hcState& state, const vector<Control>& controls) const;//@MSK: yohoho

  /** \brief Returns integrated states including covariance given a start state and controls */
  vector<hcState_With_Covariance> integrate_with_covariance(const hcState_With_Covariance& state,
                                                          const vector<Control>& controls) const;

  /** \brief Returns interpolated state at distance t in [0,1] (percentage of total path length) */
  hcState interpolate(const hcState& state, const vector<Control>& controls, double t) const;//@MSK

  /** \brief Returns integrated state given a start state, a control, and an integration step */
  inline hcState integrate_ODE(const hcState& state, const Control& control, double integration_step) const;
  vector<Configuration> q_arrays;

protected:
  /** \brief Curvature, sharpness of clothoid */
  double kappa_, sigma_sc_, sigma_cs_, gamma_;

  /** \brief Discretization of path */
  double discretization_;

  /** \brief Parameters of a hc-/cc-circle */
  GC_Circle_Param gc_circle_param_;
  // HC_CC_Circle_Param hc_cc_circle_param_StoC_;// for clothoid for straight to circle

  /** \brief Extended Kalman Filter for uncertainty propagation */
  EKF ekf_;
};

#endif
