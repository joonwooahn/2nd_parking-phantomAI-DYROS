#include "../../inc/utilities/gc_circle.h"

void GC_Circle_Param::set_param(double _kappa, double _sigma_sc, double _sigma_cs, double _radius_sc, \
                                double _mu_sc, double _sin_mu_sc, double _cos_mu_sc, double _gamma, \
                                double _radius_cs, double _mu_cs, double _sin_mu_cs, double _cos_mu_cs, \
                                double _delta_min)
{
  kappa = _kappa;
  kappa_inv = 1 / _kappa;
  sigma_sc = _sigma_sc;
  sigma_cs = _sigma_cs;
  radius_sc = _radius_sc;
  mu_sc = _mu_sc;
  sin_mu_sc = _sin_mu_sc;
  cos_mu_sc = _cos_mu_sc;
  radius_cs = _radius_cs;
  mu_cs = _mu_cs;
  sin_mu_cs = _sin_mu_cs;
  cos_mu_cs = _cos_mu_cs;
  // velo_ref = _velo_ref;
  // velo_min = _gamma * _velo_ref;
  gamma = _gamma;
  delta_min = _delta_min;
  // using_elementary = false;
}

GC_Circle::GC_Circle(const Configuration &_start, bool _left, bool _forward, bool _regular,
                           bool _straight_to_circle, const GC_Circle_Param &_param)//Constructor
{
  start = _start;
  left = _left;
  forward = _forward;
  regular = _regular;
  double radius, sin_mu, cos_mu;
  if (_straight_to_circle) {
    radius = _param.radius_sc;
    sin_mu = _param.sin_mu_sc;
    cos_mu = _param.cos_mu_sc;
  }
  else {
    radius = _param.radius_cs;
    sin_mu = _param.sin_mu_cs;
    cos_mu = _param.cos_mu_cs;
  }

  // compute xc and yc are x, y in a global frame
  double delta_x = radius * sin_mu;
  double delta_y = radius * cos_mu;
  if (left)
  {
    kappa = _param.kappa;
    kappa_inv = _param.kappa_inv;
    sigma_sc = _param.sigma_sc;
    sigma_cs = _param.sigma_cs;
    //TODO forward, backward의 의미는 진짜 차가 이 원 위에서 움직였을 때를 구해야 된다. 즉, 시작원 임에도 backward로 움직인다 해도 straight to circle!
    if (forward)
      global_frame_change(_start.x, _start.y, _start.theta, delta_x, delta_y, &xc, &yc);
    else
      global_frame_change(_start.x, _start.y, _start.theta, -delta_x, delta_y, &xc, &yc);
  }
  else
  {
    kappa = -_param.kappa;
    kappa_inv = -_param.kappa_inv;
    sigma_sc = -_param.sigma_sc;
    sigma_cs = -_param.sigma_cs;
    if (forward)
      global_frame_change(_start.x, _start.y, _start.theta, delta_x, -delta_y, &xc, &yc);
    else
      global_frame_change(_start.x, _start.y, _start.theta, -delta_x, -delta_y, &xc, &yc);
  }
  radius_sc = _param.radius_sc;
  mu_sc = _param.mu_sc;
  sin_mu_sc = _param.sin_mu_sc;
  cos_mu_sc = _param.cos_mu_sc;
  
  radius_cs = _param.radius_cs;
  mu_cs = _param.mu_cs;
  sin_mu_cs = _param.sin_mu_cs;
  cos_mu_cs = _param.cos_mu_cs;
  gamma = _param.gamma;
  delta_min = _param.delta_min;
}

GC_Circle::GC_Circle(double _xc, double _yc, bool _left, bool _forward, bool _regular,
                           const GC_Circle_Param &_param)//TODO
{
  start = Configuration(0, 0, 0, 0);
  left = _left;
  forward = _forward;
  regular = _regular;
  if (left)
  {
    kappa = _param.kappa;
    kappa_inv = _param.kappa_inv;
    sigma_sc = _param.sigma_sc;
    sigma_cs = _param.sigma_cs;
  }
  else
  {
    kappa = -_param.kappa;
    kappa_inv = -_param.kappa_inv;
    sigma_sc = -_param.sigma_sc;
    sigma_cs = -_param.sigma_cs;
  }
  xc = _xc;
  yc = _yc;
  radius_sc = _param.radius_sc;
  mu_sc = _param.mu_sc;
  sin_mu_sc = _param.sin_mu_sc;
  cos_mu_sc = _param.cos_mu_sc;

  radius_cs = _param.radius_cs;
  mu_cs = _param.mu_cs;
  sin_mu_cs = _param.sin_mu_cs;
  cos_mu_cs = _param.cos_mu_cs;
  delta_min = _param.delta_min;
}

double GC_Circle::deflection(const Configuration &q) const
{
  double alpha_c = this->start.theta;
  double alpha_q = q.theta;
  if (this->left && this->forward)
  {
    return twopify(alpha_q - alpha_c);
  }
  if (this->left && !this->forward)
  {
    return twopify(alpha_c - alpha_q);
  }
  if (!this->left && this->forward)
  {
    return twopify(alpha_c - alpha_q);
  }
  if (!this->left && !this->forward)
  {
    return twopify(alpha_q - alpha_c);
  }
}

double GC_Circle::D1(double alpha) const
{
  double fresnel_s, fresnel_c;
  double s = sqrt(2 * alpha / PI);
  fresnel(s, fresnel_s, fresnel_c);
  return cos(alpha) * fresnel_c + sin(alpha) * fresnel_s;
}

double GC_Circle::rs_circular_deflection(double delta) const
{
  // regular rs-turn
  if (this->regular)
    return delta;
  // irregular rs-turn
  else
    return (delta <= PI) ? delta : delta - TWO_PI;
}

double GC_Circle::rs_turn_length(const Configuration &q) const
{
  assert(fabs(fabs(this->kappa) - fabs(q.kappa)) < get_epsilon() &&
         fabs(fabs(this->sigma_sc) - numeric_limits<double>::max()) < get_epsilon());
  double delta = this->deflection(q);
  return fabs(this->kappa_inv * this->rs_circular_deflection(delta));
}

bool GC_Circle::gc_elementary_sharpness(const Configuration &q, double delta, double &sub_straight, double &sigma0, double &kappa0, bool straight_to_circle) const
{
  double distance = point_distance(this->start.x, this->start.y, q.x, q.y);
  // distance가 바뀌게 됨!!!!!!!!!!!!!!!
  double largeA = this->radius_sc * sin(this->mu_sc) + this->radius_cs * sin(this->mu_cs + delta);
  double largeB = this->radius_sc * cos(this->mu_sc) - this->radius_cs * cos(this->mu_cs + delta);
  double alpha_value = acos(largeA / distance);
  double beta_value = delta - alpha_value;
  double gamma_value = 0.5 * (alpha_value + beta_value);
  
  sub_straight = distance / (cos(alpha_value) + sin(alpha_value) / tan(beta_value - gamma_value));
  Configuration new_inter_point;
  if (sgn(sub_straight) >= 0.0)
  {
    new_inter_point.x = this->start.x + sub_straight * cos(this->start.theta);
    new_inter_point.y = this->start.y + sub_straight * sin(this->start.theta);
    distance = point_distance(new_inter_point.x, new_inter_point.y, q.x, q.y);
  }
  else
  {
    new_inter_point.x = q.x + sub_straight * cos(q.theta);
    new_inter_point.y = q.y + sub_straight * sin(q.theta);
    distance = point_distance(this->start.x, this->start.y, new_inter_point.x, new_inter_point.y);
  }
  sigma0 = 4 * PI * pow(this->D1(0.5 * delta), 2) / pow(distance, 2);
  kappa0 = sqrt(delta * sigma0);
  if (!this->left)
  {
    sigma0 = -sigma0;
    kappa0 = -kappa0;
  }
  return true;  
}


// bool GC_Circle::gc_elementary_sharpness(const Configuration &q, double delta, double &sub_straight, double &sigma0, double &kappa0, bool straight_to_circle) const
// {
//   double distance = point_distance(this->start.x, this->start.y, q.x, q.y);
//   // distance가 바뀌게 됨!!!!!!!!!!!!!!!
//   double largeA = this->radius_sc * sin(this->mu_sc) + this->radius_cs * sin(this->mu_cs + delta);
//   double largeB = this->radius_sc * cos(this->mu_sc) - this->radius_cs * cos(this->mu_cs + delta);
//   double alpha_value = acos(largeA / distance);
//   double beta_value = delta - alpha_value;
//   double gamma_value = 0.5 * (alpha_value + beta_value);
  
//   sub_straight = distance / (cos(alpha_value) + sin(alpha_value) / tan(beta_value - gamma_value));
//   Configuration new_inter_point;
//   if (sgn(sub_straight) >= 0.0)
//   {
//     new_inter_point.x = this->start.x + sub_straight * cos(this->start.theta);
//     new_inter_point.y = this->start.y + sub_straight * sin(this->start.theta);
//     distance = point_distance(new_inter_point.x, new_inter_point.y, q.x, q.y);
//   }
//   else
//   {
//     new_inter_point.x = q.x + sub_straight * cos(q.theta);
//     new_inter_point.y = q.y + sub_straight * sin(q.theta);
//     distance = point_distance(this->start.x, this->start.y, new_inter_point.x, new_inter_point.y);
//   }
//   sigma0 = 4 * PI * pow(this->D1(0.5 * delta), 2) / pow(distance, 2);
//   cout << "distance: " << distance << endl;
//   cout << 2 * this->D1(0.5 * delta) * sqrt(PI / this->sigma_sc) << endl;
//   double sigma00 = PI * pow(this->D1(0.5 * delta), 2) / pow(this->radius_sc * sin(0.5 * delta + this->mu_sc), 2);
//   cout << "sigma0: " << sigma0 << ", " << sigma00 << endl;
//   kappa0 = sqrt(delta * sigma0);
//   if (!this->left)
//   {
//     sigma0 = -sigma0;
//     kappa0 = -kappa0;
//   }
//   return true;  
// }

double GC_Circle::gc_circular_deflection(double delta) const
{
  double two_delta_min_twopified = twopify(this->delta_min);
  // regular cc-turn
  if (this->regular)
  {
    if (delta < two_delta_min_twopified)
      return TWO_PI + delta - two_delta_min_twopified;
    else
      return delta - two_delta_min_twopified;
  }
  // irregular cc-turn
  else
  {
    double delta_arc1, delta_arc2;
    if (delta < two_delta_min_twopified)
    {
      delta_arc1 = delta - two_delta_min_twopified;  // negative
      delta_arc2 = delta_arc1 + TWO_PI;              // positive
    }
    else
    {
      delta_arc1 = delta - two_delta_min_twopified;  // positive
      delta_arc2 = delta_arc1 - TWO_PI;              // negative
    }
    return (fabs(delta_arc1) < fabs(delta_arc2)) ? delta_arc1 : delta_arc2;
  }
// if (this->regular)
//       return delta - this->delta_min;
//   // irregular cc-turn
//   else
//   {
//     double delta_arc;
//     if (delta < this->delta_min + M_PI)
//       delta_arc = delta - this->delta_min;
//     else
//       delta_arc = TWO_PI - (delta - this->delta_min);
//     return delta_arc;
//   }
}

double GC_Circle::gc_turn_length(const Configuration &q, bool straight_to_circle) const
{
  assert(fabs(q.kappa) < get_epsilon());
  double delta = this->deflection(q);
  if (delta < get_epsilon())// just straight line
    return (straight_to_circle) ? 2 * this->radius_sc * this->sin_mu_sc: 2 * this->radius_cs * this->sin_mu_cs;

  double length_min = fabs(this->kappa / this->sigma_sc) + fabs(this->kappa / this->sigma_cs);// GC minimum length without circular arc,, 2 * kappa / sigma
  double length_default;
  if (this->using_elementary && delta < this->delta_min)// deflection minimum 보다 작은지 검사, 작은 경우 elementary path 사용
  {// 0 < delta <= delta_min
    double sub_straight, sigma0, kappa0;
    if (this->gc_elementary_sharpness(q, delta, sub_straight, sigma0, kappa0, straight_to_circle))
    {
      double length_elementary = fabs(sub_straight) + 2.0 * sqrt(delta / fabs(sigma0));
      return length_elementary;
    }
    return numeric_limits<double>::max();
  }
  else
  {
    length_default = length_min + fabs(this->kappa_inv * this->gc_circular_deflection(delta));
    return length_default;
  }
}

double GC_Circle::hc_circular_deflection(double delta, double delta_min_twopified) const
{
  // regular hc-turn
  if (this->regular)
  {
    if (delta < delta_min_twopified)
      return TWO_PI + delta - delta_min_twopified;
    else
      return delta - delta_min_twopified;
  }
  // irregular hc-turn
  else
  {
    double delta_arc1, delta_arc2;
    if (delta < delta_min_twopified)
    {
      delta_arc1 = delta - delta_min_twopified;  // negative
      delta_arc2 = delta_arc1 + TWO_PI;          // positive
    }
    else
    {
      delta_arc1 = delta - delta_min_twopified;  // positive
      delta_arc2 = delta_arc1 - TWO_PI;          // negative
    }
    return (fabs(delta_arc1) < fabs(delta_arc2)) ? delta_arc1 : delta_arc2;
  }
}

double GC_Circle::hc_turn_length(const Configuration &q, bool straight_to_circle) const//TODO
{
  assert(fabs(fabs(this->kappa) - fabs(q.kappa)) < get_epsilon());
  double delta = this->deflection(q);
  double length_clothoid, length_circle;
  if (straight_to_circle) {
    length_clothoid = fabs(this->kappa / this->sigma_sc);
    double delta_min_twopified = twopify(0.5 * pow(this->kappa, 2) / fabs(this->sigma_sc));
    length_circle = fabs(this->kappa_inv * this->hc_circular_deflection(delta, delta_min_twopified));//TODO 확인할 필요... 존재.., deflection에 따라 알맞게 값을 주는지 검사 필요
    // cout << "length circle sc: " << length_circle << endl;
  }
  else {
    length_clothoid = fabs(this->kappa / this->sigma_cs);
    double delta_min_twopified = twopify(0.5 * pow(this->kappa, 2) / fabs(this->sigma_cs));
    length_circle = fabs(this->kappa_inv * this->hc_circular_deflection(delta, delta_min_twopified));//TODO 확인할 필요... 존재.., deflection에 따라 알맞게 값을 주는지 검사 필요
    // cout << "length circle cs: " << length_circle << endl;
  }
  return length_clothoid + length_circle;
}

void GC_Circle::print(bool eol) const
{
  cout << "GC_Circle: ";
  cout << "start: ";
  start.print(false);
  if (left)
  {
    cout << ", left";
  }
  else
  {
    cout << ", right";
  }
  if (forward)
  {
    cout << ", forward";
  }
  else
  {
    cout << ", backward";
  }
  if (regular)
  {
    cout << ", regular";
  }
  else
  {
    cout << ", irregular";
  }
  cout << ", kappa: " << kappa << ", sigma_sc: " << sigma_sc << ", sigma_cs: " << sigma_cs;
  cout << ", centre: (" << xc << ", " << yc << "), radius_sc " << radius_sc << ", mu: " << mu_sc << ", radius_cs: " << radius_cs << ", mu: " << mu_cs;
  if (eol)
  {
    cout << endl;
  }
}

double center_distance(const GC_Circle &c1, const GC_Circle &c2)
{
  return sqrt(pow(c2.xc - c1.xc, 2) + pow(c2.yc - c1.yc, 2));
}

bool configuration_on_gc_circle(const GC_Circle &c, const Configuration &q, bool straight_to_circle)
{//TODOTODO
  double distance = point_distance(c.xc, c.yc, q.x, q.y);// circle 중심부터, configuration q까지...
  // 반대가 되어야함. 왜냐하면 도착 지점에 q가 존재할 경우를 찾는 것이기에...
  double radius = (straight_to_circle) ? c.radius_cs : c.radius_sc;
  double mu = (straight_to_circle) ? c.mu_cs : c.mu_sc;
  // cout << "xc: " << c.xc << "yc: " << c.yc << endl;
  // cout << "distance: " << distance << " radius : " << radius<<endl;
  if (c.forward) //시작 원이 forward일 경우, 도착 지점은 radius2로 계산
    if (fabs(distance - radius) > get_epsilon())
      return false;
  else
    if (fabs(distance - radius) > get_epsilon())
      return false;
  
  // q -> Circle end의 start 지점.
  // qg가 qs의 HC circle 내부에 있군!//TODO
  double angle = atan2(q.y - c.yc, q.x - c.xc);
  if (c.left && c.forward)
  {
    angle = angle + HALF_PI - mu;
  }
  if (c.left && !c.forward)
  {
    angle = angle + HALF_PI + mu;
  }
  if (!c.left && c.forward)
  {
    angle = angle - HALF_PI + mu;
  }
  if (!c.left && !c.forward)
  {
    angle = angle - HALF_PI - mu;
  }
  angle = twopify(angle);
  // cout << "q theta: " << q.theta << endl;
  return fabs(q.theta - angle) < get_epsilon();
}

