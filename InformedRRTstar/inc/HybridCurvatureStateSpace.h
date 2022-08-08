#ifndef HC_STEER
#define HC_STEER

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;


class HybridCurvatureStateSpace : public SE2StateSpace
{
public:
    // This is used in ReedsShepp in OMPL, but not in HC
    class ReedsSheppPath
    {
    public:
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
        ReedsSheppPath(double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                        double w = 0., double x = 0.)
                        {
                            length_[0] = t;
                            length_[1] = u;
                            length_[2] = v;
                            length_[3] = w;
                            length_[4] = x;
                            totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
                        };
        double length() const
        {
            return totalLength_;
        }
    };

    // Constructor
    HybridCurvatureStateSpace(double turningRadius = 1.0, double curvature_dot = 0.2) : rho_(turningRadius), sigma_(curvature_dot)
    {
        std::cout << "using Hybrid Curvature!" << std::endl;
        std::cout << "turning_radius is " << rho_ << std::endl;
        std::cout << "SIGMA is " << curvature_dot << std::endl;
    }

    double distance(const State *state1, const State *state2) const override;

    void interpolate(const State *from, const State *to, double t, State *state) const override;
    virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                ReedsSheppPath &path, State *state) const;

    void sanityChecks() const override
    {
        double zero = std::numeric_limits<double>::epsilon();
        double eps = .1;  // rarely such a large error will occur
        StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
    }

protected:
    virtual void interpolate(const State *from, const State *to, const ReedsSheppPath &path, double t, State *state) const;
    // virtual void interpolate(const State *from, const ReedsSheppPath &path, double t, State *state) const;//original

    /** \brief Turning radius */
    double rho_;

    /** \brief max_curvature_derivative */
    double sigma_;

    /** \brief max_curvature_derivative */
    double discretization_interval_ = 0.5;
};

/** \brief It is not used in HC, but it is needed for integrating OMPL*/
class HybridCurvatureMotionValidator : public MotionValidator
{
public:
    HybridCurvatureMotionValidator(SpaceInformation *si) : MotionValidator(si)
    {
        std::cout << "Motion Validator Constructor" << std::endl;
        defaultSettings();
    }
    HybridCurvatureMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
    {
        defaultSettings();
    }
    ~HybridCurvatureMotionValidator() override = default;
    bool checkMotion(const State *s1, const State *s2) const override;
    bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

private:
    HybridCurvatureStateSpace *stateSpace_;
    void defaultSettings();
};

#endif
