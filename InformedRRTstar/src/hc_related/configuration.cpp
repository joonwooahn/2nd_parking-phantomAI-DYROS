#include "../../inc/utilities/configuration.h"


Configuration::Configuration(double _x, double _y, double _theta, double _kappa)
{
  x = _x;
  y = _y;
  theta = twopify(_theta);
  kappa = _kappa;
}

void Configuration::print(bool eol) const
{
  cout << "(" << x << ", " << y << ", " << theta << ", " << kappa << ")";
  if (eol)
  {
    cout << endl;
  }
}

double configuration_distance(const Configuration &q1, const Configuration &q2)
{
  return point_distance(q1.x, q1.y, q2.x, q2.y);
}

bool configuration_aligned(const Configuration &q1, const Configuration &q2)
{
  if (fabs(q2.theta - q1.theta) > get_epsilon())
  {
    return false;
  }
  double angle = twopify(atan2(q2.y - q1.y, q2.x - q1.x));
  return fabs(angle - q1.theta) <= get_epsilon();
}

bool configuration_equal(const Configuration &q1, const Configuration &q2)
{
  if (fabs(q2.theta - q1.theta) > get_epsilon())
    return false;
  if (configuration_distance(q1, q2) > get_epsilon())
    return false;
  return true;
}
