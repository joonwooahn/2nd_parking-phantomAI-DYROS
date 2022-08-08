#ifndef G00_REEDS_SHEPP_STATE_SPACE_H
#define G00_REEDS_SHEPP_STATE_SPACE_H

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "configuration.h"
#include "paths.h"
#include "gc_circle.h"
#include "gc_state_space.h"
#include "gc_paths.h"
#include "steering_functions.h"
#include "utilities.h"

using namespace std;
using namespace steer;

class GC00_Reeds_Shepp_State_Space : public GC_State_Space//@MSK: parent class is HC_CC_State_Space
{
public:
  /** \brief Constructor */
  GC00_Reeds_Shepp_State_Space(double kappa, double sigma, double gamma, double discretization = 0.1);

  /** \brief Destructor */
  ~GC00_Reeds_Shepp_State_Space();

  /** \brief Returns a sequence of turns and straight lines connecting the two circles c1 and c2 */
  GC_Path* gc00_circles_rs_path(const GC_Circle& c1, const GC_Circle& c2) const;
  // HC_CC_RS_Path* hc00_circles_rs_path(const HC_CC_Circle& c1, const HC_CC_Circle& c2, const HC_CC_Circle& c3, const HC_CC_Circle& c4) const;

  /** \brief Returns a sequence of turns and straight lines connecting a start and an end configuration */
  GC_Path* gc00_reeds_shepp(const hcState& state1, const hcState& state2) const;

  // /** \brief Returns shortest path length from state1 to state2 */
  double get_distance(const hcState& state1, const hcState& state2) const;

  // /** \brief Returns controls of the shortest path from state1 to state2 */
  vector<Control> get_controls(const hcState& state1, const hcState& state2) const;
  
  pair<vector<GC_Circle>, vector<Configuration>>  get_path_configuration(const hcState& state1, const hcState& state2);
  
private:
  /** \brief Pimpl Idiom: class that contains functions to compute the families  */
  class GC00_Reeds_Shepp;

  /** \brief Pimpl Idiom: unique pointer on class with families  */
  unique_ptr<GC00_Reeds_Shepp> gc00_reeds_shepp_;

  /** \brief Parameter of a rs-circle */
  GC_Circle_Param rs_circle_param_;
};

#endif
