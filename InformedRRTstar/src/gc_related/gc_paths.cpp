#include "../../inc/utilities/gc_paths.h"

Path_for_gc::Path_for_gc(const Configuration &_start, const Configuration &_end, double _kappa, double _sigma_sc, double _sigma_cs, double _length)
{
  start = _start;
  end = _end;
  kappa = _kappa;
  sigma_sc = _sigma_sc;
  sigma_cs = _sigma_cs;
  length = _length;
}

GC_Path::GC_Path(const Configuration &_start, const Configuration &_end, hc_gc_rs::path_type _type,
                             double _kappa, double _sigma_sc, double _sigma_cs, Configuration *_qi1, Configuration *_qi2,
                             Configuration *_qi3, Configuration *_qi4, GC_Circle *_cstart, GC_Circle *_cend,
                             GC_Circle *_ci1, GC_Circle *_ci2, double _length)
  : Path_for_gc(_start, _end, _kappa, _sigma_sc, _sigma_cs, _length)
{
  type = _type;
  qi1 = _qi1;
  qi2 = _qi2;
  qi3 = _qi3;
  qi4 = _qi4;
  cstart = _cstart;
  cend = _cend;
  ci1 = _ci1;
  ci2 = _ci2;
}

GC_Path::~GC_Path()
{
  delete qi1;
  delete qi2;
  delete qi3;
  delete qi4;
  delete cstart;
  delete ci1;
  delete ci2;
  delete cend;
}

void GC_Path::print(bool eol) const
{
  cout << "HC_GC_RS_Path: type ";
  switch (type)
  {
    case hc_gc_rs::E:
      cout << "E";
      break;
    case hc_gc_rs::S:
      cout << "S";
      break;
    case hc_gc_rs::T:
      cout << "T";
      break;
    case hc_gc_rs::TT:
      cout << "TT";
      break;
    case hc_gc_rs::TcT:
      cout << "TcT";
      break;
    // Reeds-Shepp families:
    case hc_gc_rs::TcTcT:
      cout << "TcTcT";
      break;
    case hc_gc_rs::TcTT:
      cout << "TcTT";
      break;
    case hc_gc_rs::TTcT:
      cout << "TTcT";
      break;
    case hc_gc_rs::TST:
      cout << "TST";
      break;
    case hc_gc_rs::TSTcT:
      cout << "TSTcT";
      break;
    case hc_gc_rs::TcTST:
      cout << "TcTST";
      break;
    case hc_gc_rs::TcTSTcT:
      cout << "TcTSTcT";
      break;
    case hc_gc_rs::TTcTT:
      cout << "TTcTT";
      break;
    case hc_gc_rs::TcTTcT:
      cout << "TcTTcT";
      break;
    // #####################
    case hc_gc_rs::TTT:
      cout << "TTT";
      break;
    case hc_gc_rs::TcST:
      cout << "TcST";
      break;
    case hc_gc_rs::TScT:
      cout << "TScT";
      break;
    case hc_gc_rs::TcScT:
      cout << "TcScT";
      break;
    default:
      cout << "?";
      break;
  }
  cout << ", length " << length << ", configurations ";
  start.print(false);
  cout << " -> ";
  if (qi1)
  {
    qi1->print(false);
    cout << " -> ";
  }
  if (qi2)
  {
    qi2->print(false);
    cout << " -> ";
  }
  if (qi3)
  {
    qi3->print(false);
    cout << " -> ";
  }
  if (qi4)
  {
    qi4->print(false);
    cout << " -> ";
  }
  end.print(false);
  // pub_poses.
  if (eol)
  {
    cout << endl;
  }
}

void gc_rs_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
  assert(fabs(fabs(c.kappa) - fabs(q.kappa)) < get_epsilon() &&
         fabs(fabs(c.sigma_cs) - numeric_limits<double>::max()) < get_epsilon());
  double delta = c.deflection(q);
  double length_arc = fabs(c.kappa_inv) * c.rs_circular_deflection(delta);
  int d = direction(c.forward, order);

  Control arc;
  arc.delta_s = d * length_arc;
  arc.kappa = c.kappa;
  arc.sigma = 0.0;
  controls.push_back(arc);
  return;
}

void gc_hc_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
  // cout << "radius to q: " << point_distance(c.xc, c.yc, q.x, q.y) << endl;
  // cout << "radius to q: " << point_distance(c.xc, c.yc, c.start.x, c.start.y) << endl;
  // order의 의미는 arc 먼저 나오고 clothoid인지, 그 반대인지를 의미...
  assert(fabs(fabs(c.kappa) - fabs(q.kappa)) < get_epsilon());
  double delta = c.deflection(q);
  double length_sc = fabs(c.kappa / c.sigma_sc);
  double length_cs = fabs(c.kappa / c.sigma_cs);
  double delta_min_twopified = (order) ? twopify(0.5 * pow(c.kappa, 2) / fabs(c.sigma_sc)) : twopify(0.5 * pow(c.kappa, 2) / fabs(c.sigma_cs));
  double length_arc = fabs(c.kappa_inv) * c.hc_circular_deflection(delta, delta_min_twopified);
  int d = direction(c.forward, order);
  
  Control clothoid, arc;
  if (order)//straight to circle
  {
    clothoid.delta_s = d * length_sc;
    clothoid.kappa = 0.0;
    clothoid.sigma = c.sigma_sc;
    controls.push_back(clothoid);
  }

  arc.delta_s = d * length_arc;
  arc.kappa = c.kappa;
  arc.sigma = 0.0;
  controls.push_back(arc);

  if (!order)
  {
    clothoid.delta_s = d * length_cs;
    clothoid.kappa = c.kappa;
    clothoid.sigma = -c.sigma_cs;
    controls.push_back(clothoid);
  }
  return;
}

bool gc_elementary_controls(const GC_Circle &c, const Configuration &q, double delta, bool order,
                            bool straight_to_circle, vector<Control> &controls)
{
  double sub_straight, sigma0, kappa0;
  if (c.gc_elementary_sharpness(q, delta, sub_straight, sigma0, kappa0, straight_to_circle))
  {
      double length = fabs(kappa0 / sigma0);
      int d = direction(c.forward, order);
      Control straight, clothoid1, clothoid2;
    if (sgn(sub_straight) >= 0) {
      straight.delta_s = d * fabs(sub_straight);
      straight.kappa = 0.0;
      straight.sigma = 0.0;
      controls.push_back(straight);

      clothoid1.delta_s = d * length;
      clothoid1.kappa = 0.0;
      clothoid1.sigma = sigma0;
      controls.push_back(clothoid1);

      clothoid2.delta_s = d * length;
      clothoid2.kappa = kappa0;// clothoid 시작점의 kappa
      clothoid2.sigma = -sigma0;
      controls.push_back(clothoid2);
    }
    else {
      double length = fabs(kappa0 / sigma0);
      int d = direction(c.forward, order);
      Control straight, clothoid1, clothoid2;

      clothoid1.delta_s = d * length;
      clothoid1.kappa = 0.0;
      clothoid1.sigma = sigma0;
      controls.push_back(clothoid1);

      clothoid2.delta_s = d * length;
      clothoid2.kappa = kappa0;// clothoid 시작점의 kappa
      clothoid2.sigma = -sigma0;
      controls.push_back(clothoid2);

      straight.delta_s = d * fabs(sub_straight);
      straight.kappa = 0.0;
      straight.sigma = 0.0;
      controls.push_back(straight);  
    }
    return true;
  }
  return false;
}

void gc_default_controls(const GC_Circle &c, const Configuration &q, double delta, bool order,
                         vector<Control> &controls)
{//진짜 중요:: straight -> circle에서 clothoid는 sigma_min
  double length_sc = fabs(c.kappa / c.sigma_sc);
  double length_cs = fabs(c.kappa / c.sigma_cs);
  double length_arc = fabs(c.kappa_inv) * c.gc_circular_deflection(delta);
  int d = direction(c.forward, order);
  Control clothoid1, arc, clothoid2;
  clothoid1.delta_s = d * length_sc;
  clothoid1.kappa = 0.0;
  clothoid1.sigma = c.sigma_sc;
  controls.push_back(clothoid1);

  arc.delta_s = d * length_arc;
  arc.kappa = c.kappa;
  arc.sigma = 0.0;
  controls.push_back(arc);

  clothoid2.delta_s = d * length_cs;
  clothoid2.kappa = c.kappa;
  clothoid2.sigma = -c.sigma_cs;
  controls.push_back(clothoid2);
  return;
}

void gc_turn_controls(const GC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
  assert(fabs(q.kappa) < get_epsilon());
  double delta = c.deflection(q);
  if (delta < get_epsilon())
  {
    if (order)
      straight_controls(c.start, q, controls);
    else
      straight_controls(q, c.start, controls);
    return;
  }
  else if (c.using_elementary && delta < c.delta_min)// deflection minimum 보다 작은지 검사, 작은 경우 elementary path 사용
  {
    vector<Control> controls_elementary, controls_default;
    if (gc_elementary_controls(c, q, delta, order, true, controls_elementary))
    {
      gc_default_controls(c, q, delta, order, controls_default);
      double length_elementary = 
          accumulate(controls_elementary.begin(), controls_elementary.end(), 0.0,
                      [](double sum, const Control &control) { return sum + fabs(control.delta_s); });
      double length_default =
          accumulate(controls_default.begin(), controls_default.end(), 0.0,
                    [](double sum, const Control &control) { return sum + fabs(control.delta_s); });
      (length_elementary < length_default) ?
          controls.insert(controls.end(), controls_elementary.begin(), controls_elementary.end()) :
          controls.insert(controls.end(), controls_default.begin(), controls_default.end());
      return;//todo... 흠...
    }
  }
  // delta >= 2 * delta_min
  else
  {
    gc_default_controls(c, q, delta, order, controls);
    return;
  }
}
