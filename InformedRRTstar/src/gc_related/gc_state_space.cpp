#include "../../inc/utilities/gc_state_space.h"

GC_State_Space::GC_State_Space(double kappa, double sigma_cs, double gamma, double discretization)
  : kappa_(kappa), sigma_cs_(sigma_cs), gamma_(gamma), discretization_(discretization) 
{
  assert(kappa > 0.0 && sigma_cs > 0.0 && discretization > 0.0);
  double velo_ref = 1.0;//1.6667 if 5.0 [km/h]
  double velo_min = gamma * velo_ref;//3.6 [km/h]
  sigma_sc_ = gamma * sigma_cs;

  // intermediate configuration after first clothoid
  double length_min_sc = (1 / sigma_sc_) * kappa;//
  double length_min_cs = (1 / sigma_cs_) * kappa;// circle to straight line
  double length_min = length_min_sc + length_min_cs;
  /** \brief **/
  // sigma는 원래 sigma_max 값. 즉, circle->straight 클로소이드의 기울기, radius2, mu2 가 여기 해당, 작은 원에 해당. 
  // 원래 hc, cc의 반지름 등등은 여기에 해당.
  // sigma_min은 straight _> clothoid,, radius, mu 여기 해당. 큰 원에 해당 
  double x_i, y_i, theta_i;
  double x_j, y_j, theta_j;
  if (length_min_sc > get_epsilon())// 0 이상.
  {
    double kappa_i;
    end_of_clothoid(0, 0, 0, 0, sigma_sc_, 1, length_min_sc, &x_i, &y_i, &theta_i, &kappa_i);// @MSK this function computs q_i in the paper
    // [x, y, theta, kappa, simga, direction, length_min, x_i, y_i, theta_i, kappa_i]
  }
  else
  {// @MSK: epsilon < length_min
    x_i = 0;
    y_i = 0;
    theta_i = 0;
  }
  if (length_min_cs > get_epsilon())// 0 이상.
  {
    double kappa_j;
    end_of_clothoid(0, 0, 0, 0, sigma_cs_, 1, length_min_cs, &x_j, &y_j, &theta_j, &kappa_j);// @MSK this function computs q_i in the paper
    // [x, y, theta, kappa, simga, direction, length_min, x_i, y_i, theta_i, kappa_i]
  }
  else
  {// @MSK: epsilon < length_min
    x_j = 0;
    y_j = 0;
    theta_j = 0;
  }

  // constructor에서는 단순히 radius와 mu, mu2를 계산하는 것 뿐이다.
  // it assumes that init_pose is {0, 0, 0}
  // radius
  double xc, yc; // equation (7) in HC paper
  xc = x_i - sin(theta_i) / kappa;
  yc = y_i + cos(theta_i) / kappa;
  double radius_sc = point_distance(xc, yc, 0.0, 0.0);

  // mu
  double mu_sc = atan(fabs(xc / yc)); // equation (9)
  double sin_mu_sc = sin(mu_sc);
  double cos_mu_sc = cos(mu_sc);// 
  // cout << "xc: " << xc << " yc: " << yc << endl;
  xc = x_j - sin(theta_j) / kappa;
  yc = y_j + cos(theta_j) / kappa;
  // cout << "xc: " << xc << " yc: " << yc << endl;
  double radius_cs = point_distance(xc, yc, 0.0, 0.0);
  // mu2
  double mu_cs = atan(fabs(xc / yc)); // equation (9)
  double sin_mu_cs = sin(mu_cs);
  double cos_mu_cs = cos(mu_cs);
  // cout << "what i need:: " << radius_cs * cos_mu_cs << " , " << radius_cs * sin_mu_cs << endl;

  // delta_min
  double delta_min = 0.5 * kappa * length_min;
  // assign
  gc_circle_param_.set_param(kappa, sigma_sc_, sigma_cs, radius_sc, \
                              mu_sc, sin_mu_sc, cos_mu_sc, gamma_, \
                              radius_cs, mu_cs, sin_mu_cs, cos_mu_cs, delta_min);
}


void GC_State_Space::set_filter_parameters(const Motion_Noise &motion_noise,
                                              const Measurement_Noise &measurement_noise, const Controller &controller)
{
  ekf_.set_parameters(motion_noise, measurement_noise, controller);
}

vector<hcState> GC_State_Space::get_path(const hcState &state1, const hcState &state2) const
{
  vector<Control> controls = get_controls(state1, state2);
  return integrate(state1, controls);
}

// //@MSK
vector<hcState_With_Covariance> GC_State_Space::get_path_with_covariance(const hcState_With_Covariance &state1,
                                                                          const hcState &state2) const
{
  vector<Control> controls = get_controls(state1.state, state2);//@MSK: controls 의 경우는 해당 HC path를 따라가기 위한 {delta_s, kappa, sigma}
  return integrate_with_covariance(state1, controls);
}

vector<hcState> GC_State_Space::integrate(const hcState &state, const vector<Control> &controls) const
{
  vector<hcState> path;
  hcState state_curr, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  // cout << "n states: " << n_states << endl;

  path.reserve(n_states + 3);
  // push back first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;
  state_curr.kappa = controls.front().kappa;
  state_curr.d = sgn(controls.front().delta_s);
  path.push_back(state_curr);// 처음 state를 집어 넣는다.

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double kappa(control.kappa);//현재 kappa 값.
    double s_seg(0.0);
    double integration_step(0.0);
    // push_back current state if curvature discontinuity
    if (fabs(kappa - state_curr.kappa) > get_epsilon())
    {
      state_curr.kappa = kappa;
      state_curr.d = sgn(delta_s);
      path.push_back(state_curr);
    }

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      state_next = integrate_ODE(state_curr, control, integration_step);
      path.push_back(state_next);
      state_curr = state_next;
    }
  }
  return path;
}

vector<hcState_With_Covariance> GC_State_Space::integrate_with_covariance(const hcState_With_Covariance &state,
                                                                           const vector<Control> &controls) const
{
  vector<hcState_With_Covariance> path_with_covariance;
  hcState_With_Covariance state_curr, state_pred, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  path_with_covariance.reserve(n_states + 3);
  // push back first state// @MSK: state = start_state
  state_curr.state.x = state.state.x;
  state_curr.state.y = state.state.y;
  state_curr.state.theta = state.state.theta;
  state_curr.state.kappa = controls.front().kappa;
  state_curr.state.d = sgn(controls.front().delta_s);
  for (int i = 0; i < 16; i++)
  {
    state_curr.Sigma[i] = state.Sigma[i];
    state_curr.Lambda[i] = state.Lambda[i];
    state_curr.covariance[i] = state.covariance[i];
  }
  path_with_covariance.push_back(state_curr);// @MSK: start_state is pushed back

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double kappa(control.kappa);
    double s_seg(0.0);
    double integration_step(0.0);
    
    // push_back current state if curvature discontinuity
    if (fabs(kappa - state_curr.state.kappa) > get_epsilon())
    {
      // @MSK: 전후진 전환 횟수 만큼 본 if statement가 실행.
      state_curr.state.kappa = kappa;
      state_curr.state.d = sgn(delta_s);
      path_with_covariance.push_back(state_curr);
    }
    // cout << "total waypt num: " << ceil(abs_delta_s / discretization_) << endl;
    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      // predict
      state_pred.state = integrate_ODE(state_curr.state, control, integration_step);
      ekf_.predict(state_curr, control, integration_step, state_pred);
      // update
      state_next.state = state_pred.state;
      ekf_.update(state_pred, state_next);

      path_with_covariance.push_back(state_next);
      state_curr.state = state_next.state;
      for (int i = 0; i < 16; i++)
      {
        state_curr.Sigma[i] = state_next.Sigma[i];
        state_curr.Lambda[i] = state_next.Lambda[i];
        state_curr.covariance[i] = state_next.covariance[i];
      }
    }
  }
  return path_with_covariance;
}

hcState GC_State_Space::interpolate(const hcState &state, const vector<Control> &controls, double t) const
{
  hcState state_curr, state_next;

  // get first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;
  state_curr.kappa = controls.front().kappa;
  state_curr.d = sgn(controls.front().delta_s);
  // get arc length at t
  double s_path(0.0);//@MSK: total length of the Path
  double s_inter(0.0);
  for (const auto &control : controls)
  {
    s_path += fabs(control.delta_s);
  }
  // 전체 패쓰의 길이를 0에서 1로 생각 후, 스테이트를 리턴하는 역할
  if (t <= 0.0)
    return state_curr;
  else if (t > 1.0)
    s_inter = s_path;
  else
    s_inter = t * s_path;

  double s(0.0);
  bool interpolated = false;
  for (const auto &control : controls)
  {// 콘트롤은 클로소이드 직진, 등등으로 이루어짐.
    if (interpolated)
      break;

    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double kappa(control.kappa);//현재 필요한 control kappa 값 update.
    double s_seg(0.0);
    double integration_step(0.0);
    // update current state if curvature discontinuity@// curvature가 불연속적일 때, current state를 그에 맞게 변화 시켜 줌.
    if (fabs(kappa - state_curr.kappa) > get_epsilon())
    {
      state_curr.kappa = kappa;
      state_curr.d = sgn(delta_s);
    }

    s += abs_delta_s;
    if (s > s_inter)//현재 state까지의 총 경로 길이가 interpolate 길이보다 클 때, BREAK~~
    {
      abs_delta_s = abs_delta_s - (s - s_inter);
      interpolated = true;
    }

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      state_next = integrate_ODE(state_curr, control, integration_step);
      state_curr = state_next;
    }
  }
  return state_curr;
}

inline hcState GC_State_Space::integrate_ODE(const hcState &state, const Control &control, double integration_step) const
{
  hcState state_next;
  double sigma(control.sigma);
  double d(sgn(control.delta_s));
  if (fabs(sigma) > get_epsilon())
  {
    end_of_clothoid(state.x, state.y, state.theta, state.kappa, sigma, d, integration_step, &state_next.x,
                    &state_next.y, &state_next.theta, &state_next.kappa);
    state_next.d = d;
  }
  else
  {
    if (fabs(state.kappa) > get_epsilon())
    {
      end_of_circular_arc(state.x, state.y, state.theta, state.kappa, d, integration_step, &state_next.x, &state_next.y,
                          &state_next.theta);
      state_next.kappa = state.kappa;
      state_next.d = d;
    }
    else
    {
      end_of_straight_line(state.x, state.y, state.theta, d, integration_step, &state_next.x, &state_next.y);
      // cout << "straight state: " << state_next.x << ", " << state_next.y << endl;
      state_next.theta = state.theta;
      state_next.kappa = state.kappa;
      state_next.d = d;
    }
  }
  return state_next;
}
