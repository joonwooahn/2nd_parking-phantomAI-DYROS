#ifndef EKF_HPP
#define EKF_HPP

#include <iostream>

#include <Eigen/Dense>

#include "steering_functions.h"
#include "utilities.h"

using namespace std;

typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 2> Matrix32d;
typedef Eigen::Matrix<double, 2, 3> Matrix23d;

using namespace steer;
class EKF
{
public:
  /** Constructor */
  EKF();

  /** \brief Sets the parameters required by the EKF */
  void set_parameters(const Motion_Noise &motion_noise, const Measurement_Noise &measurement_noise,
                      const Controller &_controller);

  /** \brief Converts a covariance given by a double array to an Eigen matrix */
  Matrix3d covariance_to_eigen(const double covariance[16]) const;

  /** \brief Converts a covariance given by an Eigen matrix to a double array */
  void eigen_to_covariance(const Matrix3d &covariance_eigen, double covariance[16]) const;

  /** \brief Computes the Jacobians of the motion equations with respect to the state and control */
  void get_motion_jacobi(const hcState &state, const Control &control, double integration_step, Matrix3d &F_x,
                        Matrix32d &F_u) const;

  /** \brief Computes the Jacobian of the observation equations with respect to the state */
  Matrix3d get_observation_jacobi() const;

  /** \brief Returns the motion covariance in control space */
  Matrix2d get_motion_covariance(const hcState &state, const Control &control, double integration_step) const;

  /** \brief Returns the observation covariance */
  Matrix3d get_observation_covariance() const;

  /** \brief Returns the gain of the controller */
  Matrix23d get_controller_gain(const Control &control) const;

  /** \brief Returns the rotation matrix from a global frame to a local frame */
  Matrix3d get_rotation_matrix(double angle) const;

  /** \brief Predicts the covariances based on the paper:
      Rapidly-exploring random belief trees for motion planning under uncertainty, A. Bry and N. Roy, IEEE ICRA 2011 */
  void predict(const hcState_With_Covariance &state, const Control &control, double integration_step,
              hcState_With_Covariance &state_pred) const;

  /** \brief Predicts the covariances */
  void update(const hcState_With_Covariance &state_pred, hcState_With_Covariance &state_corr) const;

  /** \brief Overload operator new for fixed-size vectorizable Eigen member variable */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  /** \brief Motion noise */
  Motion_Noise motion_noise_;

  /** \brief Measurement noise */
  Measurement_Noise measurement_noise_;

  /** \brief Feedback controller */
  Controller controller_;

  /** \brief Identity matrix */
  Matrix3d I_;
};

#endif
