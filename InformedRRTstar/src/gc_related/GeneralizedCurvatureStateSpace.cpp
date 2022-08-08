#include "../../inc/GeneralizedCurvatureStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>

#include "../../inc/utilities/steering_functions.h"
#include "../../inc/utilities/gc00_reeds_shepp_state_space.h"
#include <vector>

#define using_HC true

using namespace ompl::base;

double GeneralizedCurvatureStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *s_from = static_cast<const StateType *>(state1);
    const auto *s_to = static_cast<const StateType *>(state2);
    
    steer::hcState start_wout_curv;
    start_wout_curv.x = s_from->getX(); //start.state.x;
    start_wout_curv.y = s_from->getY(); //start.state.y;
    start_wout_curv.theta = s_from->getYaw(); //start.state.theta;
    start_wout_curv.kappa = 0.0;// initial point curvature
    start_wout_curv.d = 0.0;
    steer::hcState goal_wout_curv;
    goal_wout_curv.x = s_to->getX();
    goal_wout_curv.y = s_to->getY();
    goal_wout_curv.theta = s_to->getYaw();
    goal_wout_curv.kappa = 0.0;// goal point curvature
    goal_wout_curv.d = 0.0;

    GC00_Reeds_Shepp_State_Space state_space(1/rho_, sigma_, gamma_, discretization_interval_);

    return state_space.get_distance(start_wout_curv, goal_wout_curv);
}

//Computes the state that lies at time t in [0, 1] on the segment that connects from state to to state.
void GeneralizedCurvatureStateSpace::interpolate(const State *from, const State *to, const double t,
                                                   State *state) const
{
    bool firstTime = true;
    ReedsSheppPath _;
    interpolate(from, to, t, firstTime, _, state);
}

void GeneralizedCurvatureStateSpace::interpolate(const State *from, const State *to, const double t, bool &firstTime,
                                                   ReedsSheppPath &path, State *state) const
{
    interpolate(from, to, path, t, state);
}

void GeneralizedCurvatureStateSpace::interpolate(const State *from, const State *to, const ReedsSheppPath &path, double t,
                                                   State *state) const
{
    // std::cout << "interpolate! " << std::endl;
    auto *s = allocState()->as<StateType>();
    const auto *s_from = static_cast<const StateType *>(from);
    const auto *s_to = static_cast<const StateType *>(to);

    double kappa_max_ = 1.0 / rho_;//max_curvature
    double sigma_max_ = sigma_;//max_curvature_derivative
    double gamma_value = gamma_;
    double discretization_ = discretization_interval_;
    
    steer::hcState start_wout_curv;
    start_wout_curv.x = s_from->getX(); //start.state.x;
    start_wout_curv.y = s_from->getY(); //start.state.y;
    start_wout_curv.theta = s_from->getYaw(); //start.state.theta;
    start_wout_curv.kappa = 0.0;// initial point curvature
    start_wout_curv.d = 0.0;

    steer::hcState goal_wout_curv;// @MSK goal with a straight
    goal_wout_curv.x = s_to->getX();
    goal_wout_curv.y = s_to->getY();
    goal_wout_curv.theta = s_to->getYaw();; //goal.theta;
    goal_wout_curv.kappa = 0.0;// goal point curvature
    goal_wout_curv.d = 0.0;

    GC00_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, gamma_value, discretization_);

    vector<Control> path_control = state_space.get_controls(start_wout_curv, goal_wout_curv);
    steer::hcState state_curr_;
    state_curr_ = state_space.interpolate(start_wout_curv, path_control, t);

    state->as<StateType>()->setX(state_curr_.x);//@MSK: state의 값을 변경
    state->as<StateType>()->setY(state_curr_.y);//: 앞서 나누줬던 minimum turning radius를 다시 곱.
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(state_curr_.theta);
    freeState(s);
}

void GeneralizedCurvatureMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<GeneralizedCurvatureStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw ompl::Exception("No state space for motion validator");
}

bool GeneralizedCurvatureMotionValidator::checkMotion(const State *s1, const State *s2,
                                                        std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    std::cout << "checkMotion" << std::endl;
    bool result = true, firstTime = true;
    GeneralizedCurvatureStateSpace::ReedsSheppPath _;//temporal path, it is not used...
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, _, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, _, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, _, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool GeneralizedCurvatureMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    GeneralizedCurvatureStateSpace::ReedsSheppPath _;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, _, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
