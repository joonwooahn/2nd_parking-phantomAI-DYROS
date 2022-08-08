#ifndef STEERING_FUNCTIONS_H
#define STEERING_FUNCTIONS_H


namespace steer
{
  /** \brief Description of a kinematic car's state */
  struct hcState
  {
    /** \brief Position in x of the robot */
    double x;

    /** \brief Position in y of the robot */
    double y;

    /** \brief Orientation of the robot */
    double theta;

    /** \brief Curvature at position (x,y) */
    double kappa;

    /** \Driving direction {-1,0,1} */
    double d;
  };

  /** \brief Description of a kinematic car's state with covariance */
  struct hcState_With_Covariance
  {
    /** \brief Expected hcState of the robot */
    hcState state;

    /** \brief Covariance of the state estimation due to motion and measurement noise */
    double Sigma[16] = { 0.0 };

    /** \brief Covariance of the state estimate due to the absence of measurements */
    double Lambda[16] = { 0.0 };

    /** \brief Covariance of the state given by Sigma + Lambda: (x_x      x_y      x_theta      x_kappa
                                                                 y_x      y_y      y_theta      y_kappa
                                                                theta_x  theta_y  theta_theta  theta_kappa
                                                                kappa_x  kappa_y  kappa_theta  kappa_kappa) */
    double covariance[16] = { 0.0 };
  };

  /** \brief Description of a path segment with its corresponding control inputs */
  struct Control
  {
    /** \brief Signed arc length of a segment */
    double delta_s;

    /** \brief Curvature at the beginning of a segment */
    double kappa;

    /** \brief Sharpness (derivative of curvature with respect to arc length) of a segment */
    double sigma;
  };

  /** \brief Parameters of the motion noise model according to the book:
      Probabilistic Robotics, S. Thrun and others, MIT Press, 2006, p. 127-128 and p.204-206. */
  struct Motion_Noise
  {
    /** \brief Variance in longitudinal direction: alpha1*delta_s*delta_s + alpha2*kappa*kappa  */
    double alpha1;
    double alpha2;

    /** \brief Variance in lateral direction: alpha3*delta_s*delta_s + alpha4*kappa*kappa */
    double alpha3;
    double alpha4;
  };

  /** \brief Parameters of the measurement noise */
  struct Measurement_Noise
  {
    /** \brief Standard deviation of localization in x */
    double std_x;

    /** \brief Standard deviation of localization in y */
    double std_y;

    /** \brief Standard deviation of localization in theta */
    double std_theta;
  };

  /** \brief Parameters of the feedback controller */
  struct Controller
  {
    /** \brief Weight on longitudinal error */
    double k1;

    /** \brief Weight on lateral error */
    double k2;

    /** \brief Weight on heading error */
    double k3;
  };
}
  
#endif
