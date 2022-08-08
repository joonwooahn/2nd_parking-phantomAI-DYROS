#ifndef OMPL_BASE_SPACES_HC00_REEDS_SHEPP_STATE_SPACE_
#define OMPL_BASE_SPACES_HC00_REEDS_SHEPP_STATE_SPACE_


#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "configuration.h"
#include "hc_cc_circle.h"
#include "hc_cc_state_space.h"
#include "paths.h"
#include "steering_functions.h"
#include "utilities.h"

using namespace std;

class HC00_Reeds_Shepp_State_Space : public HC_CC_State_Space
{
    public:
        
        /** \brief Constructor */
        HC00_Reeds_Shepp_State_Space(double kappa, double sigma, double discretization = 0.1);

        /** \brief Destructor */
        ~HC00_Reeds_Shepp_State_Space();

        /** \brief Returns a sequence of turns and straight lines connecting the two circles c1 and c2 */
        HC_CC_RS_Path* hc00_circles_rs_path(const HC_CC_Circle& c1, const HC_CC_Circle& c2) const;

        /** \brief Returns a sequence of turns and straight lines connecting a start and an end configuration */
        HC_CC_RS_Path* hc00_reeds_shepp(const hcState& state1, const hcState& state2) const;

        /** \brief Returns shortest path length from state1 to state2 */
        double get_distance(const hcState& state1, const hcState& state2) const;

        /** \brief Returns controls of the shortest path from state1 to state2 */
        vector<Control> get_controls(const hcState& state1, const hcState& state2) const;

        /** \brief Pimpl Idiom: class that contains functions to compute the families  */
        class HC00_Reeds_Shepp;

        /** \brief Pimpl Idiom: unique pointer on class with families  */
        unique_ptr<HC00_Reeds_Shepp> hc00_reeds_shepp_;

        /** \brief Parameter of a rs-circle */
        HC_CC_Circle_Param rs_circle_param_;

        /** \brief Returns interpolated state at distance t in [0,1] (percentage of total path length) */
        // hcState interpolate(const hcState& state, const vector<Control>& controls, double t) const;//@MSK
        // hcState printStuff() {
        //     HC_CC_State_Space::printStuff(); // base class의 함수 호출
        // }

    // double distance(const State *state1, const State *state2) const override;

    // void interpolate(const State *from, const State *to, double t, State *state) const override;
    // virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
    //                          ReedsSheppPath &path, State *state) const;

    // void sanityChecks() const override
    // {
    //     double zero = std::numeric_limits<double>::epsilon();
    //     double eps = .1;  // rarely such a large error will occur
    //     StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
    // }

    // /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    // ReedsSheppPath reedsShepp(const State *state1, const State *state2) const;

// protected:
//     virtual void interpolate(const State *from, const ReedsSheppPath &path, double t, State *state) const;

//     /** \brief Turning radius */
//     double rho_;
};

/** \brief A Reeds-Shepp motion validator that only uses the state validity checker.
    Motions are checked for validity at a specified resolution.

    This motion validator is almost identical to the DiscreteMotionValidator
    except that it remembers the optimal ReedsSheppPath between different calls to
    interpolate. */
// class ReedsSheppMotionValidator : public MotionValidator
// {
// public:
//     ReedsSheppMotionValidator(SpaceInformation *si) : MotionValidator(si)
//     {
//         defaultSettings();
//     }
//     ReedsSheppMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
//     {
//         defaultSettings();
//     }
//     ~ReedsSheppMotionValidator() override = default;
//     bool checkMotion(const State *s1, const State *s2) const override;
//     bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

// private:
//     ReedsSheppStateSpace *stateSpace_;
//     void defaultSettings();
// };


#endif
