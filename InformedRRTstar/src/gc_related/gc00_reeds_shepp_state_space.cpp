#include "../../inc/utilities/gc00_reeds_shepp_state_space.h"

#define CC_REGULAR false

class GC00_Reeds_Shepp_State_Space::GC00_Reeds_Shepp
{
private:
  GC00_Reeds_Shepp_State_Space *parent_;

public:
  explicit GC00_Reeds_Shepp(GC00_Reeds_Shepp_State_Space *parent)
  {
    parent_ = parent;
  }

  double distance = 0.0;
  double angle = 0.0;

//   // ##### TT ###################################################################
  bool TT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)// 두 원의 turn 방향은 달라야됨
    {
      return false;
    }
    if (c1.forward == c2.forward)// 두 원의 방향은 달라야 됨.
    {
      return false;
    }
    return fabs(distance - sqrt(pow(c1.radius_sc * c1.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) //done
                              + pow(c1.radius_sc * c1.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs, 2))) < get_epsilon();// HC circle이 접하는지 검사

  }

  void TT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q) const
  {
    //todo: 무조건 시작 지점에서는 sc 이기 때문에 상관 없을 것 같긴 한데.. 고려해보기!
    // c1은 무조건 straight to circle 원임!
    double c_distance = point_distance(c1.xc, c1.yc, c2.xc, c2.yc);
    double alpha = acos((pow(c1.radius_cs, 2) + pow(c_distance, 2) - pow(c2.radius_sc, 2)) / (2.0 * c_distance * c1.radius_cs));
    double cangle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double angle;// MSK!!! iPad
    if (c1.left)
    {
      if (c1.forward) angle = cangle - alpha;
      else angle = cangle + alpha;
    }
    else
    {
      if (c1.forward) angle = cangle + alpha;
      else angle = cangle - alpha;
    }
    double x, y;
    global_frame_change(c1.xc, c1.yc, angle, c1.radius_cs, 0.0, &x, &y);//첫번째 원의 radius_cs 만큼 떨어진 곳에 q 존재.
    double theta;
    
    if (c1.left)
    {
      if (c1.forward)
      {
        theta = angle - c1.mu_cs + HALF_PI;// 처음 원이 cs 이므로...
      }
      else
      {
        theta = angle + c1.mu_cs + HALF_PI;
      }
    }
    else
    {
      if (c1.forward)
      {
        theta = angle + c1.mu_cs - HALF_PI;
      }
      else
      {
        theta = angle - c1.mu_cs - HALF_PI;
      }
    }
    *q = new Configuration(x, y, theta, 0);
  }

  double TT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                 Configuration **q) const
  {
    TT_tangent_circles(c1, c2, q);//c1과 c2가 TT로 되어, 그 접점인 q를 구하는 함수.
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    return (*cstart)->gc_turn_length(**q, true) + (*cend)->gc_turn_length(**q, false);//각 CC circle의 length를 구하여 진행.
  }

  // ##### TcT ##################################################################
  bool TcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return fabs(distance - 2 * fabs(c1.kappa_inv)) < get_epsilon();// Turn 후, backward이므로 RS circle과 겹치는지 확인.
  }

  void TcT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q) const
  {
    double distance = center_distance(c1, c2);
    double delta_x = 0.5 * distance;
    double delta_y = 0.0;
    double angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double x, y, theta;
    if (c1.left)
    {
      if (c1.forward)
      {
        theta = angle + HALF_PI;
        global_frame_change(c1.xc, c1.yc, angle, delta_x, delta_y, &x, &y);
      }
      else
      {
        theta = angle + HALF_PI;
        global_frame_change(c1.xc, c1.yc, angle, delta_x, -delta_y, &x, &y);
      }
    }
    else
    {
      if (c1.forward)
      {
        theta = angle - HALF_PI;
        global_frame_change(c1.xc, c1.yc, angle, delta_x, -delta_y, &x, &y);
      }
      else
      {
        theta = angle - HALF_PI;
        global_frame_change(c1.xc, c1.yc, angle, delta_x, delta_y, &x, &y);
      }
    }
    *q = new Configuration(x, y, theta, c1.kappa);
  }

  double TcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                  Configuration **q) const
  {
    TcT_tangent_circles(c1, c2, q);
    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);
    return (*cstart)->hc_turn_length(**q, true) + (*cend)->hc_turn_length(**q, false);
  }

// //   // ##### Reeds-Shepp families: ################################################

  // ##### TcTcT ################################################################
  bool TcTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)// 방향이 달라야함
    {
      return false;
    }
    return distance <= 4.0 * fabs(c1.kappa_inv);// Fig. 6 in HC paper -- triangular inequality
  }

  void TcTcT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r = 2.0 * fabs(c1.kappa_inv);
    double delta_x = 0.5 * distance;
    double delta_y = sqrt(pow(r, 2) - pow(delta_x, 2));//h in Fig. 6 at HC paper
    double x, y;

    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);// 원이 위에 있는 경우,,
    GC_Circle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);// 원이 아래에 있는 경우,,
    GC_Circle tgt2(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, c2, q2);
    TcT_tangent_circles(c1, tgt2, q3);
    TcT_tangent_circles(tgt2, c2, q4);
  }

  double TcTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2, GC_Circle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    GC_Circle *middle1, *middle2;// rs circle이 2가지 경우가 존재(?). 이 둘 중에 길이가 작은 애의 길이를 리턴
    middle1 = new GC_Circle(*qa, !c1.left, !c1.forward, true, true, parent_->rs_circle_param_);
    middle2 = new GC_Circle(*qc, !c1.left, !c1.forward, true, true, parent_->rs_circle_param_);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);

    // select shortest connection// TODO
    double length1 = (*cstart)->hc_turn_length(*qa, true) + middle1->rs_turn_length(*qb) + (*cend)->hc_turn_length(*qb, false);
    double length2 = (*cstart)->hc_turn_length(*qc, true) + middle2->rs_turn_length(*qd) + (*cend)->hc_turn_length(*qd, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      return length1;
    }
    else
    {
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

//   // ##### TcTT #################################################################
  bool TcTT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)// turn direction은 같아야 됨.
    {
      return false;
    }
    if (c1.forward != c2.forward)// forward/backward direction은 같아야됨...
    {
      return false;
    }
    double two_gc_circle_distance = sqrt(pow(c1.radius_sc * c1.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) //done
                                  + pow(c1.radius_sc * c1.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs, 2));
    // 아래 조건을 만족하면, 존재. 첫번째 조건은 triangular inequality.
    return (distance <= two_gc_circle_distance + 2.0 * fabs(c1.kappa_inv)) && (distance >= two_gc_circle_distance - 2.0 * fabs(c1.kappa_inv));
  }

  void TcTT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;//제 2 코사인...
    double r1 = 2.0 * fabs(c1.kappa_inv);// RS-radius
    double r2 = sqrt(pow(c1.radius_sc * c1.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) //done
                                  + pow(c1.radius_sc * c1.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs, 2));// same as two_gc_circle_distance
    double delta_x = (pow(r1, 2) + pow(distance, 2) - pow(r2, 2)) / (2.0 * distance);
    double delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));
    double x, y;

    // y의 부호가 다르므로, 원의 위치가 다른 형태. 즉, 중간에 긴 원이 위에 존재 혹은 아래에 존재?
    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);//상단 원
    TcT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, c2, q2);
    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);//하단 원
    TcT_tangent_circles(c1, tgt2, q3);
    TT_tangent_circles(tgt2, c2, q4);
  }

  double TcTT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2, GC_Circle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    GC_Circle *middle1, *middle2;
    middle1 = new GC_Circle(*qb, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);
    middle2 = new GC_Circle(*qd, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);//cc circle

    // select shortest connection
    double length1 = (*cstart)->hc_turn_length(*qa, true) + middle1->hc_turn_length(*qa, false) + (*cend)->gc_turn_length(*qb, false);
    double length2 = (*cstart)->hc_turn_length(*qc, true) + middle2->hc_turn_length(*qc, false) + (*cend)->gc_turn_length(*qd, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      return length1;
    }
    else
    {
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

//   // ##### TTcT #################################################################
  bool TTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {// TcTT와 동일.
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)// 방향이 같아야됨...
    {
      return false;
    }//TODO
    double two_gc_circle_distance = sqrt(pow(c1.radius_sc * c1.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) //done
                                  + pow(c1.radius_sc * c1.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs, 2));
    return (distance <= two_gc_circle_distance + 2.0 * fabs(c1.kappa_inv)) && (distance >= two_gc_circle_distance - 2.0 * fabs(c1.kappa_inv));
  }

  vector<pair<double, double>> TTcT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = sqrt(pow(c1.radius_sc * c1.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) //done
                                  + pow(c1.radius_sc * c1.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs, 2));// same as two_gc_circle_distance
    double r2 = 2.0 * fabs(c1.kappa_inv);

    // double beta = acos((pow(r1, 2) + pow(distance, 2) - pow(r2, 2)) / (2.0 * distance * r1));
    double delta_x = (pow(r1, 2) + pow(distance, 2) - pow(r2, 2)) / (2.0 * distance);
    double delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));
    // double delta_x = r1;
    // double delta_y = 0.0;
    double x, y;

    vector<pair<double, double>> middle_coordinates;
    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);
    middle_coordinates.push_back(make_pair(x, y));
    TT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, c2, q2);

    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);
    middle_coordinates.push_back(make_pair(x, y));
    TT_tangent_circles(c1, tgt2, q3);
    TcT_tangent_circles(tgt2, c2, q4);
    return middle_coordinates;
  }

  double TTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2, GC_Circle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    vector<pair<double, double>> middle_coordinates;
    middle_coordinates = TTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    GC_Circle *middle1, *middle2;
    middle1 = new GC_Circle(*qa, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);
    middle2 = new GC_Circle(*qc, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);
    // middle1->xc = middle_coordinates[0].first; middle1->yc = middle_coordinates[0].second;
    // middle2->xc = middle_coordinates[1].first; middle2->yc = middle_coordinates[1].second;
    
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2);
    // select shortest connection
    double length1 = (*cstart)->gc_turn_length(*qa, true) + middle1->hc_turn_length(*qb, true) + (*cend)->hc_turn_length(*qb, false);
    double length2 = (*cstart)->gc_turn_length(*qc, true) + middle2->hc_turn_length(*qd, true) + (*cend)->hc_turn_length(*qd, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      return length1;
    }
    else
    {
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TST ##################################################################
  bool TiST_exists(const GC_Circle &c1, const GC_Circle &c2) const//s자 커브를 그림
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)// 하나는 forward, 하나는 backward
    {
      return false;
    }
    return (distance >= sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2)));
    // return (distance >= c1.radius + c1.radius2);
  }

  bool TeST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= sqrt(pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2)));
    // return (distance >= c1.radius * c1.sin_mu + c1.radius2 * c1.sin_mu2);
  }

  bool TST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TiST_exists(c1, c2) || TeST_exists(c1, c2);
  }

  void TiST_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1,
                            Configuration **q2) const
  {
    double distance = center_distance(c1, c2);
    double angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

    double alpha = asin( (c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc) / distance);//TODO
    double c1_delta_x = c1.radius_cs * c1.sin_mu_cs;// Fig. 10 bottom at CC paper 참고.
    double c1_delta_y = c1.radius_cs * c1.cos_mu_cs;
    double c2_delta_x = c2.radius_sc * c2.sin_mu_sc;
    double c2_delta_y = c2.radius_sc * c2.cos_mu_sc;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, c1_delta_x, -c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);//각 CC circle의 qg를 의미
      global_frame_change(c2.xc, c2.yc, theta, -c2_delta_x, c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;// 결국 직선에서 출발하는 것이기에 처음에 도달하는 곳의 mu와 radius는 달라짐
      global_frame_change(c1.xc, c1.yc, theta, c1_delta_x, c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, -c2_delta_x, -c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, c1_delta_x, c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta, -c2_delta_x, -c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, c1_delta_x, -c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, -c2_delta_x, c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
  }

  void TeST_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1,
                            Configuration **q2) const
  {
    double cdistance = center_distance(c1, c2);
    double cos_diff = c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs;
    double alpha = asin(cos_diff / cdistance);
    double theta = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    
    double c1_delta_x = c1.radius_cs * c1.sin_mu_cs;// Fig. 10 bottom at CC paper 참고.
    double c1_delta_y = c1.radius_cs * c1.cos_mu_cs;
    double c2_delta_x = c2.radius_sc * c2.sin_mu_sc;
    double c2_delta_y = c2.radius_sc * c2.cos_mu_sc;
    double x, y;
    if (c1.left && c1.forward)
    {//TODO shit
      global_frame_change(c1.xc, c1.yc, theta - alpha, c1_delta_x, -c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta - alpha, 0);
      global_frame_change(c2.xc, c2.yc, theta - alpha, -c2_delta_x, -c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta - alpha, 0);
    }
    if (c1.left && !c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta - alpha, c1_delta_x, c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta - alpha + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta - alpha, -c2_delta_x, c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta - alpha + PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta - alpha, c1_delta_x, c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta - alpha, -c2_delta_x, c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta - alpha, c1_delta_x, -c1_delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta - alpha + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta - alpha, -c2_delta_x, -c2_delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta - alpha + PI, 0);
    }
  }

  double TiST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    TiST_tangent_circles(c1, c2, q1, q2);
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->gc_turn_length(**q2, false);// CC turn length + qg1-qg2 길이 + CC turn length
  }

  double TeST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    TeST_tangent_circles(c1, c2, q1, q2);
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->gc_turn_length(**q2, false);
  }

  double TST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                  Configuration **q1, Configuration **q2) const
  {
    if (TiST_exists(c1, c2))
    {
      return TiST_path(c1, c2, cstart, cend, q1, q2);
    }
    if (TeST_exists(c1, c2))
    {
      return TeST_path(c1, c2, cstart, cend, q1, q2);
    }
    return numeric_limits<double>::max();
  }

  // ##### TSTcT ################################################################
  bool TiSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance >=
            sqrt(pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc + 2 * fabs(c1.kappa_inv), 2) 
              + pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)));
  }

  bool TeSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    // return (distance >= 2 * fabs(c1.kappa_inv) + c1.radius * c1.sin_mu + c1.radius2 * c1.sin_mu2);
    return (distance >= sqrt(pow(2 * fabs(c1.kappa_inv) + c1.radius_cs * c1.sin_mu_cs +  c2.radius_sc * c2.sin_mu_sc, 2) +
                            pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2)));
  }

  bool TSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TiSTcT_exists(c1, c2) || TeSTcT_exists(c1, c2);
  }

  double TiSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    double theta = angle;
    // 닮음 이용
    double delta_y = (2.0 * fabs(c2.kappa_inv) * (c1.radius_cs * c1.cos_mu_cs +  c2.radius_sc * c2.cos_mu_sc) / distance);
    // double delta_y = (2 * 2 * c2.radius * c2.cos_mu) / (fabs(c2.kappa) * distance);
    double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    // global_frame_change(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);//TODO 내 생각...
    global_frame_change(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);

    TiST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2);//todo
    *ci = new GC_Circle(**q2, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);

    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*ci)->hc_turn_length(**q3, true) +
           (*cend)->hc_turn_length(**q3, false);
  }

  double TeSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    // double theta = angle;
    // double delta_y = (2.0 * fabs(c2.kappa_inv) * (c2.radius_sc * c2.cos_mu_sc -  c1.radius_cs * c1.cos_mu_cs) / distance);
    // double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double two_rs_radius = 2.0 * fabs(c2.kappa_inv);
    double cdistance = center_distance(c1, c2);
    double radius_to_tgt = sqrt(pow(c2.radius_sc * c2.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) 
                                + pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2));
    double beta = acos( (pow(cdistance, 2) + pow(radius_to_tgt, 2) - pow(two_rs_radius, 2)) / (2.0 * cdistance * radius_to_tgt));
    double delta_x = radius_to_tgt;
    double delta_y = 0.0;

    double x, y;
    global_frame_change(c1.xc, c1.yc, angle - beta, delta_x, delta_y, &x, &y);
    // global_frame_change(c2.xc, c2.yc, angle - beta, -delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);
    
    TeST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2);
    *ci = new GC_Circle(**q2, c1.left, c1.forward, true, true, parent_->gc_circle_param_);

    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*ci)->hc_turn_length(**q3, true) +
           (*cend)->hc_turn_length(**q3, false);
  }

  double TSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    if (TiSTcT_exists(c1, c2))
    {
      return TiSTcT_path(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    if (TeSTcT_exists(c1, c2))
    {
      return TeSTcT_path(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTST ################################################################
  bool TcTiST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance >=
            sqrt(pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c1.sin_mu_sc + 2.0 * fabs(c1.kappa_inv), 2) 
              + pow(c1.radius_cs * c1.cos_mu_cs + c1.radius_sc * c1.cos_mu_sc, 2)));
  }

  bool TcTeST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance >= sqrt(pow(2.0 * fabs(c1.kappa_inv) + c1.radius_cs * c1.sin_mu_cs +  c1.radius_sc * c1.sin_mu_sc, 2) +
                            pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2)));
    // return (distance >= 2 * (fabs(c1.kappa_inv) + c1.radius * c1.sin_mu));
  }

  bool TcTST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TcTiST_exists(c1, c2) || TcTeST_exists(c1, c2);
  }

  double TcTiST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    double theta = angle;
    double delta_y = 2.0 * fabs(c2.kappa_inv) * (c2.radius_sc * c2.cos_mu_sc + c1.radius_cs * c1.cos_mu_cs) / distance;
    double delta_x = sqrt(pow(2.0 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    // global_frame_change(c1.xc, c1.yc, theta, delta_x, +delta_y, &x, &y);// TODO...theta에 따라 바뀔 수 있을 것 같은데..., 
    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c2.left, !c2.forward, c2.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, c2, q2, q3);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    *ci = new GC_Circle(**q2, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);

    return (*cstart)->hc_turn_length(**q1, true) + (*ci)->hc_turn_length(**q1, false) + configuration_distance(**q2, **q3) +
           (*cend)->gc_turn_length(**q3, false);
  }

  double TcTeST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    // double theta = angle;// Original... TODO!! 확인 필요.;.. 밑에는 바꾼 것.
    // double delta_y = 2.0 * fabs(c2.kappa_inv) * (c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs) / distance;
    // double delta_x = sqrt(pow(2.0 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    double two_rs_radius = 2.0 * fabs(c2.kappa_inv);
    double cdistance = center_distance(c1, c2);
    double radius_to_tgt = sqrt(pow(c2.radius_sc * c2.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) 
                                + pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2));
    double beta = acos( (pow(cdistance, 2) + pow(two_rs_radius, 2) - pow(radius_to_tgt, 2)) / (2.0 * cdistance * two_rs_radius));
    double delta_x = two_rs_radius;
    double delta_y = 0.0;

    global_frame_change(c1.xc, c1.yc, angle - beta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt1(x, y, c2.left, !c2.forward, c2.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, c2, q2, q3);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    *ci = new GC_Circle(**q2, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);

    return (*cstart)->hc_turn_length(**q1, true) + (*ci)->hc_turn_length(**q1, false) + configuration_distance(**q2, **q3) +
           (*cend)->gc_turn_length(**q3, false);
  }

  double TcTST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci) const
  {
    if (TcTiST_exists(c1, c2))
    {
      return TcTiST_path(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    if (TcTeST_exists(c1, c2))
    {
      return TcTeST_path(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTSTcT ##############################################################
  bool TcTiSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >=
            sqrt(pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc + 4.0 * fabs(c1.kappa_inv), 2) + 
                pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)));
    // return (distance >=
    //         sqrt(pow(2 * c1.radius, 2) + 16 * c1.radius * c1.sin_mu * fabs(c1.kappa_inv) + pow(4 * c1.kappa_inv, 2)));
  }

  bool TcTeSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= sqrt(pow(4.0 * fabs(c1.kappa_inv) + c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2) + 
                              pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2)));
    // return (distance >= 4 * fabs(c1.kappa_inv) + 2 * c1.radius * c1.sin_mu);
  }

  bool TcTSTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TcTiSTcT_exists(c1, c2) || TcTeSTcT_exists(c1, c2);
  }

  double TcTiSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                       GC_Circle **ci1, GC_Circle **ci2) const
  {
    double theta = angle;
    double delta_y = 2.0 * fabs(c1.kappa_inv) * (c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc) / distance;
    double delta_x = sqrt(pow(2.0 * c1.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);
    *ci1 = new GC_Circle(**q2, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);
    *ci2 = new GC_Circle(**q3, !c2.left, c2.forward, true, true, parent_->gc_circle_param_);

    return (*cstart)->hc_turn_length(**q1, true) + (*ci1)->hc_turn_length(**q1, false) + configuration_distance(**q2, **q3) +
           (*ci2)->hc_turn_length(**q4, true) + (*cend)->hc_turn_length(**q4, false);
  }

  double TcTeSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                       GC_Circle **ci1, GC_Circle **ci2) const
  {// mu1, mu2가 있기에 단순히 2 * fabs(c1.kappa_inv)로 하면 안될 것 같은데, 왜냐하면 angle 방향에 tgt1 중심이 걸쳐있지 않으므로...
    double theta = angle;//TODO
    double delta_y = 2.0 * fabs(c1.kappa_inv) * (c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs) / distance;
    double delta_x = 2.0 * sqrt(pow(2.0 * c1.kappa_inv, 2) - pow(delta_y, 2));
    // double delta_x = 2 * fabs(c1.kappa_inv);
    // double delta_y = 0;
    double x, y;

    // double two_rs_radius = 2.0 * fabs(c2.kappa_inv);
    // double cdistance = center_distance(c1, c2);
    // double radius_to_tgt = sqrt(pow(c2.radius_sc * c2.sin_mu_sc + c1.radius_cs * c1.sin_mu_cs, 2) 
    //                             + pow(c2.radius_sc * c2.cos_mu_sc - c1.radius_cs * c1.cos_mu_cs, 2));
    // double beta = acos( (pow(cdistance, 2) + pow(two_rs_radius, 2) - pow(radius_to_tgt, 2)) / (2.0 * cdistance * two_rs_radius));
    // double delta_x = two_rs_radius;
    // double delta_y = 0.0;

    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);
    *ci1 = new GC_Circle(**q2, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);
    *ci2 = new GC_Circle(**q3, !c2.left, c2.forward, true, true, parent_->gc_circle_param_);

    return (*cstart)->hc_turn_length(**q1, true) + (*ci1)->hc_turn_length(**q1, false) + configuration_distance(**q2, **q3) +
           (*ci2)->hc_turn_length(**q4, true) + (*cend)->hc_turn_length(**q4, false);
  }

  double TcTSTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                      GC_Circle **ci1, GC_Circle **ci2) const
  {
    if (TcTiSTcT_exists(c1, c2))
    {
      return TcTiSTcT_path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
    }
    if (TcTeSTcT_exists(c1, c2))
    {
      return TcTeSTcT_path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
    }
    return numeric_limits<double>::max();
  }

//   // ##### TTcTT ###############################################################
  bool TTcTT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    double two_gc_circle_distance = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2));
    return (distance <= 2.0 * two_gc_circle_distance + 2.0 * fabs(c1.kappa_inv));
  }

  void TTcTT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1, r2, delta_x, delta_y, x, y;
    r1 = 2.0 * fabs(c1.kappa_inv);
    r2 = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2));
    
    if (distance < 2.0 * r2 - 2.0 * fabs(c1.kappa_inv))//todo...
    {
      delta_x = (distance + r1) / 2.0;
      delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
    }
    else
    {
      delta_x = (distance - r1) / 2.0;
      delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
    }
    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);//TODO
    GC_Circle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c2.left, !c2.forward, c2.regular, parent_->gc_circle_param_);

    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt3(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    GC_Circle tgt4(x, y, !c2.left, !c2.forward, c2.regular, parent_->gc_circle_param_);

    TT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, tgt2, q2);
    TT_tangent_circles(tgt2, c2, q3);

    TT_tangent_circles(c1, tgt3, q4);
    TcT_tangent_circles(tgt3, tgt4, q5);
    TT_tangent_circles(tgt4, c2, q6);
  }

  double TTcTT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci1,
                    GC_Circle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TTcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    GC_Circle *middle1, *middle2, *middle3, *middle4;
    middle1 = new GC_Circle(*qa, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);
    middle2 = new GC_Circle(*qc, !c2.left, c2.forward, true, false, parent_->gc_circle_param_);
    middle3 = new GC_Circle(*qd, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);
    middle4 = new GC_Circle(*qf, !c2.left, c2.forward, true, false, parent_->gc_circle_param_);

    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, true, parent_->gc_circle_param_);

    // select shortest connection
    double length1 = (*cstart)->gc_turn_length(*qa, true) + middle1->hc_turn_length(*qb, true) + middle2->hc_turn_length(*qb, false) +
                     (*cend)->gc_turn_length(*qc, false);
    double length2 = (*cstart)->gc_turn_length(*qd, true) + middle3->hc_turn_length(*qe, true) + middle4->hc_turn_length(*qe, false) +
                     (*cend)->gc_turn_length(*qf, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *q3 = qc;
      *ci1 = middle1;
      *ci2 = middle2;
      delete qd;
      delete qe;
      delete qf;
      delete middle3;
      delete middle4;
      return length1;
    }
    else
    {
      *q1 = qd;
      *q2 = qe;
      *q3 = qf;
      *ci1 = middle3;
      *ci2 = middle4;
      delete qa;
      delete qb;
      delete qc;
      delete middle1;
      delete middle2;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTTcT ###############################################################
  bool TcTTcT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }//todo//와이...
    double two_gc_circle_distance = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2));
    return (distance <= 4.0 * fabs(c1.kappa_inv) + two_gc_circle_distance) && (distance >= 4.0 * fabs(c1.kappa_inv) - two_gc_circle_distance);
    // return (distance <= 4 * fabs(c1.kappa_inv) + 2 * c1.radius) && (distance >= 4 * fabs(c1.kappa_inv) - 2 * c1.radius);
  }

  void TcTTcT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                              Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {//제 2 코사인..
    double theta = angle;
    double x, y;
    double r1 = 2.0 * fabs(c1.kappa_inv);
    double r2 = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2)) / 2; // c1.radius (at original)
    double delta_x1 = (pow(r1, 2) + pow(distance / 2.0, 2) - pow(r2, 2)) / distance;
    double delta_y1 = sqrt(pow(r1, 2) - pow(delta_x1, 2));
    double delta_x2 = (pow(r1, 2) + pow(distance / 2.0, 2) - pow(r2, 2)) / distance;
    double delta_y2 = sqrt(pow(r1, 2) - pow(delta_x2, 2));

    global_frame_change(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
    GC_Circle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);

    global_frame_change(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
    GC_Circle tgt3(x, y, !c1.left, !c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
    GC_Circle tgt4(x, y, !c2.left, c2.forward, c2.regular, parent_->gc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, tgt2, q2);
    TcT_tangent_circles(tgt2, c2, q3);

    TcT_tangent_circles(c1, tgt3, q4);
    TT_tangent_circles(tgt3, tgt4, q5);
    TcT_tangent_circles(tgt4, c2, q6);
  }

  double TcTTcT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, GC_Circle **ci1,
                     GC_Circle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TcTTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    GC_Circle *middle1, *middle2, *middle3, *middle4;
    middle1 = new GC_Circle(*qb, !c1.left, c1.forward, true, false, parent_->gc_circle_param_);
    middle2 = new GC_Circle(*qb, c1.left, !c1.forward, true, true, parent_->gc_circle_param_);
    middle3 = new GC_Circle(*qe, !c1.left, c1.forward, true, false,parent_->gc_circle_param_);
    middle4 = new GC_Circle(*qe, c1.left, !c1.forward, true, true, parent_->gc_circle_param_);

    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);

    // select shortest connection
    double length1 = (*cstart)->hc_turn_length(*qa, true) + middle1->hc_turn_length(*qa, false) + middle2->hc_turn_length(*qc, true) +
                     (*cend)->hc_turn_length(*qc, false);
    double length2 = (*cstart)->hc_turn_length(*qd, true) + middle3->hc_turn_length(*qd, false) + middle4->hc_turn_length(*qf, true) +
                     (*cend)->hc_turn_length(*qf, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *q3 = qc;
      *ci1 = middle1;
      *ci2 = middle2;
      delete qd;
      delete qe;
      delete qf;
      delete middle3;
      delete middle4;
      return length1;
    }
    else
    {
      *q1 = qd;
      *q2 = qe;
      *q3 = qf;
      *ci1 = middle3;
      *ci2 = middle4;
      delete qa;
      delete qb;
      delete qc;
      delete middle1;
      delete middle2;
      return length2;
    }
    return numeric_limits<double>::max();
  }

//   // ############################################################################

//   // ##### TTT ##################################################################
  bool TTT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    
    double two_gc_circle_distance = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2));
    return distance <= 2.0 * two_gc_circle_distance;
  }

  void TTT_tangent_circles(const GC_Circle &c1, const GC_Circle &c2, Configuration **q1, Configuration **q2,
                           Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r;
    r = sqrt(pow(c1.radius_cs * c1.cos_mu_cs + c2.radius_sc * c2.cos_mu_sc, 2)
                     + pow(c1.radius_cs * c1.sin_mu_cs + c2.radius_sc * c2.sin_mu_sc, 2));
    double delta_x = 0.5 * distance;
    double delta_y = sqrt(pow(r, 2) - pow(delta_x, 2));
    double x, y;
  
    global_frame_change(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    GC_Circle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);
    global_frame_change(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    GC_Circle tgt2(x, y, !c1.left, c1.forward, c1.regular, parent_->gc_circle_param_);

    TT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, c2, q2);
    TT_tangent_circles(c1, tgt2, q3);
    TT_tangent_circles(tgt2, c2, q4);
  }

  double TTT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                  Configuration **q1, Configuration **q2, GC_Circle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    GC_Circle *middle1, *middle2;
    middle1 = new GC_Circle(*qa, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);
    middle2 = new GC_Circle(*qc, !c1.left, c1.forward, true, true, parent_->gc_circle_param_);

    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);

    // select shortest connection
    double length1 = (*cstart)->gc_turn_length(*qa, true) + middle1->gc_turn_length(*qb, true) + (*cend)->gc_turn_length(*qb, false);
    double length2 = (*cstart)->gc_turn_length(*qc, true) + middle2->gc_turn_length(*qd, true) + (*cend)->gc_turn_length(*qd, false);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      return length1;
    }
    else
    {
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TcST ################################################################
  bool TciST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)// turn 방향은 달라야됨...
    {
      return false;
    }
    if (c1.forward != c2.forward)// 방향이 같아야됨...
    {
      return false;
    }// 피타고라스 처럼 생김//TODO
    return distance >= sqrt(pow(c2.radius_sc * c2.sin_mu_sc, 2) + pow(c2.radius_sc * c2.cos_mu_sc + fabs(c1.kappa_inv), 2));
    // return distance >= sqrt(pow(c1.radius * c1.sin_mu, 2) + pow(c1.radius * c1.cos_mu + fabs(c1.kappa_inv), 2));
  }

  bool TceST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }//TODO
    return distance >= sqrt(pow(c2.radius_sc * c2.sin_mu_sc, 2) + pow(c2.radius_sc * c2.cos_mu_sc - fabs(c1.kappa_inv), 2));
  }

  bool TcST_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TciST_exists(c1, c2) || TceST_exists(c1, c2);
  }

  double TciST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c2.radius_sc * c2.cos_mu_sc + fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = c2.radius_sc * c2.sin_mu_sc;
    double delta_y2 = c2.radius_sc * c2.cos_mu_sc;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    return (*cstart)->hc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->gc_turn_length(**q2, false);
  }

  double TceST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c2.radius_sc * c2.cos_mu_sc - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = c2.radius_sc * c2.sin_mu_sc;
    double delta_y2 = c2.radius_sc * c2.cos_mu_sc;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2.start, c2.left, c2.forward, CC_REGULAR, false, parent_->gc_circle_param_);
    return (*cstart)->hc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->gc_turn_length(**q2, false);
  }

  double TcST_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    if (TciST_exists(c1, c2))
    {
      return TciST_path(c1, c2, cstart, cend, q1, q2);
    }
    if (TceST_exists(c1, c2))
    {
      return TceST_path(c1, c2, cstart, cend, q1, q2);
    }
    return numeric_limits<double>::max();
  }

  // ##### TScT #################################################################
  bool TiScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)// 턴 방향 다름.
    {
      return false;
    }
    if (c1.forward != c2.forward)// 방향 동일.
    {
      return false;
    }
    return distance >= sqrt(pow(c1.radius_cs * c1.sin_mu_cs, 2) + pow(c1.radius_cs * c1.cos_mu_cs + fabs(c1.kappa_inv), 2));
  }

  bool TeScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(c1.radius_cs * c1.sin_mu_cs, 2) + pow(c1.radius_cs * c1.cos_mu_cs - fabs(c1.kappa_inv), 2));
  }

  bool TScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TiScT_exists(c1, c2) || TeScT_exists(c1, c2);
  }

  double TiScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius_cs * c1.cos_mu_cs + fabs(c2.kappa_inv)) / distance);
    double delta_x1 = c1.radius_cs * c1.sin_mu_cs;
    double delta_y1 = c1.radius_cs * c1.cos_mu_cs;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2);
    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->hc_turn_length(**q2, false);
  }

  double TeScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius_cs * c1.cos_mu_cs - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = c1.radius_cs * c1.sin_mu_cs;
    double delta_y1 = c1.radius_cs * c1.cos_mu_cs;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, 0);
      global_frame_change(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    *cstart = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, parent_->gc_circle_param_);
    *cend = new GC_Circle(c2);
    return (*cstart)->gc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->hc_turn_length(**q2, false);
  }

  double TScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    if (TiScT_exists(c1, c2))
    {
      return TiScT_path(c1, c2, cstart, cend, q1, q2);
    }
    if (TeScT_exists(c1, c2))
    {
      return TeScT_path(c1, c2, cstart, cend, q1, q2);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcScT ################################################################
  bool TciScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance > 2 * fabs(c1.kappa_inv);
  }

  bool TceScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance >= get_epsilon();
  }

  bool TcScT_exists(const GC_Circle &c1, const GC_Circle &c2) const
  {
    return TciScT_exists(c1, c2) || TceScT_exists(c1, c2);
  }

  double TciScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2) const
  {
     /** \todo **/
    double alpha = asin(2.0 / (fabs(c1.kappa) * distance));// asin( kappa_inv / (distance/2) )
    double delta_x = 0.0;
    double delta_y = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);
    return (*cstart)->hc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->hc_turn_length(**q2, false);
  }

  double TceScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                     Configuration **q1, Configuration **q2) const
  {
    double theta = angle;
    double delta_x = 0.0;
    double delta_y = fabs(c1.kappa_inv);
    double x, y;
    if (c1.left && c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + PI, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + PI, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      global_frame_change(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      global_frame_change(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    *cstart = new GC_Circle(c1);
    *cend = new GC_Circle(c2);
    return (*cstart)->hc_turn_length(**q1, true) + configuration_distance(**q1, **q2) + (*cend)->hc_turn_length(**q2, false);
  }

  double TcScT_path(const GC_Circle &c1, const GC_Circle &c2, GC_Circle **cstart, GC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    if (TciScT_exists(c1, c2))
    {
      return TciScT_path(c1, c2, cstart, cend, q1, q2);
    }
    if (TceScT_exists(c1, c2))
    {
      return TceScT_path(c1, c2, cstart, cend, q1, q2);
    }
    return numeric_limits<double>::max();
  }
};

// ############################################################################
GC00_Reeds_Shepp_State_Space::GC00_Reeds_Shepp_State_Space(double kappa, double sigma, double gamma, double discretization)
  : GC_State_Space(kappa, sigma, gamma, discretization)
  , gc00_reeds_shepp_{ unique_ptr<GC00_Reeds_Shepp>(new GC00_Reeds_Shepp(this)) }
{
  rs_circle_param_.set_param(kappa_, numeric_limits<double>::max(), numeric_limits<double>::max(), 1.0 / kappa_,
                             0.0, 0.0, 1.0, 0.0, //double _mu_sc, double _sin_mu_sc, double _cos_mu_sc, double _gamma,
                             1.0 / kappa_, 0.0, 0.0, 1.0, 0.0);// double _radius_cs, double _mu_cs, double _sin_mu_cs, double _cos_mu_cs, double _delta_min);
}

GC00_Reeds_Shepp_State_Space::~GC00_Reeds_Shepp_State_Space() = default;


GC_Path *GC00_Reeds_Shepp_State_Space::gc00_circles_rs_path(const GC_Circle &c1, const GC_Circle &c2) const
{
  // @MSK: 경로를 만드는 곳
  // table containing the lengths of the paths, the intermediate configurations and circles
  // int number_of_candidate_path = 18;
  int number_of_candidate_path = nb_hc_gc_rs_paths;
  double length[number_of_candidate_path];//nb_hc_gc_rs_paths = 18, this value in path.hpp// counting a whole number of steering function family
  double_array_init(length, number_of_candidate_path, numeric_limits<double>::max());
  Configuration *qi1[number_of_candidate_path];
  pointer_array_init((void **)qi1, number_of_candidate_path);
  Configuration *qi2[number_of_candidate_path];
  pointer_array_init((void **)qi2, number_of_candidate_path);
  Configuration *qi3[number_of_candidate_path];
  pointer_array_init((void **)qi3, number_of_candidate_path);
  Configuration *qi4[number_of_candidate_path];
  pointer_array_init((void **)qi4, number_of_candidate_path);
  GC_Circle *cstart[number_of_candidate_path];
  pointer_array_init((void **)cstart, number_of_candidate_path);
  GC_Circle *ci1[number_of_candidate_path];
  pointer_array_init((void **)ci1, number_of_candidate_path);
  GC_Circle *ci2[number_of_candidate_path];
  pointer_array_init((void **)ci2, number_of_candidate_path);
  GC_Circle *cend[number_of_candidate_path];
  pointer_array_init((void **)cend, number_of_candidate_path);

  // precomputations
  gc00_reeds_shepp_->distance = center_distance(c1, c2);// l(C1, C2) in Fig.10 at CC paper.
  gc00_reeds_shepp_->angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

  // case Equal
  if (configuration_equal(c1.start, c2.start))// 원 중점이 동일한지 판단
  {
    length[hc_gc_rs::E] = 0;
    goto label_end;
  }
  // case S forwards
  if (configuration_aligned(c1.start, c2.start))// 두 원의 시작 지점, qs와 qg의 angle 차이 X
  {
    length[hc_gc_rs::S] = configuration_distance(c1.start, c2.start);
    goto label_end;
  }
  // case S backwards
  if (configuration_aligned(c2.start, c1.start))
  {
    length[hc_gc_rs::S] = configuration_distance(c2.start, c1.start);
    goto label_end;
  }
  // case T
  if (configuration_on_gc_circle(c1, c2.start, true))// // Elementary path!?
  {// 6.254593592464422, y: 2.2795203591158795, th: 0.25*M_PI
    cstart[hc_gc_rs::T] = new GC_Circle(c1.start, c1.left, c1.forward, CC_REGULAR, true, gc_circle_param_);
    length[hc_gc_rs::T] = cstart[hc_gc_rs::T]->gc_turn_length(c2.start, true);
    goto label_end;
  }
  // case TT
  if (gc00_reeds_shepp_->TT_exists(c1, c2))
  {// x: 13.1538426515, y: 13.1538426515, th: 0.0*M_PI;
    length[hc_gc_rs::TT] =// 이때는 자연스럽게, CC와 동일. CC circle 2개의 path length를 더하여 return.
        gc00_reeds_shepp_->TT_path(c1, c2, &cstart[hc_gc_rs::TT], &cend[hc_gc_rs::TT], &qi1[hc_gc_rs::TT]);
  }
  // case TcT
  if (gc00_reeds_shepp_->TcT_exists(c1, c2))
  {//x: 1.98938061592244 + 10.0 + 0.998668, y = 5.13257400901962 - 5.03329 th = 1.0*M_PI;
    length[hc_gc_rs::TcT] =// 이때는 자연스럽게, CC와 동일. CC circle 2개의 path length를 더하여 return.
        gc00_reeds_shepp_->TcT_path(c1, c2, &cstart[hc_gc_rs::TcT], &cend[hc_gc_rs::TcT], &qi1[hc_gc_rs::TcT]);
  }

  // ##### Reeds-Shepp families: ############################################
  // case TcTcT
  if (gc00_reeds_shepp_->TcTcT_exists(c1, c2))//TODO...
  {// (-2, 4, 0.8pi)/진짜 안됨... 왜 안되냐... 시붕... (10.0, -1.0, -0.72pi)
    length[hc_gc_rs::TcTcT] =// Fig.6 in HC paper
        gc00_reeds_shepp_->TcTcT_path(c1, c2, &cstart[hc_gc_rs::TcTcT], &cend[hc_gc_rs::TcTcT], &qi1[hc_gc_rs::TcTcT],
                                      &qi2[hc_gc_rs::TcTcT], &ci1[hc_gc_rs::TcTcT]);
  }
  // case TcTT
  if (gc00_reeds_shepp_->TcTT_exists(c1, c2))
  {// (2.0, 10.0, 0.15pi)
    length[hc_gc_rs::TcTT] =
        gc00_reeds_shepp_->TcTT_path(c1, c2, &cstart[hc_gc_rs::TcTT], &cend[hc_gc_rs::TcTT], &qi1[hc_gc_rs::TcTT],
                                     &qi2[hc_gc_rs::TcTT], &ci1[hc_gc_rs::TcTT]);
  }
  // case TTcT
  if (gc00_reeds_shepp_->TTcT_exists(c1, c2))
  {
    length[hc_gc_rs::TTcT] =
        gc00_reeds_shepp_->TTcT_path(c1, c2, &cstart[hc_gc_rs::TTcT], &cend[hc_gc_rs::TTcT], &qi1[hc_gc_rs::TTcT],
                                     &qi2[hc_gc_rs::TTcT], &ci1[hc_gc_rs::TTcT]);
  }
  // case TST; Turn - Straight - Turn
  if (gc00_reeds_shepp_->TST_exists(c1, c2))
  {// i와 e의 차이는 방향 차이. i일 경우, 곡률의 방향 변화. e일 경우, U자 모양.
    length[hc_gc_rs::TST] = gc00_reeds_shepp_->TST_path(c1, c2, &cstart[hc_gc_rs::TST], &cend[hc_gc_rs::TST],
                                                        &qi1[hc_gc_rs::TST], &qi2[hc_gc_rs::TST]);
  }
  // case TSTcT
  if (gc00_reeds_shepp_->TSTcT_exists(c1, c2))
  {
    length[hc_gc_rs::TSTcT] =
        gc00_reeds_shepp_->TSTcT_path(c1, c2, &cstart[hc_gc_rs::TSTcT], &cend[hc_gc_rs::TSTcT], &qi1[hc_gc_rs::TSTcT],
                                      &qi2[hc_gc_rs::TSTcT], &qi3[hc_gc_rs::TSTcT], &ci1[hc_gc_rs::TSTcT]);
  }
  // case TcTST
  if (gc00_reeds_shepp_->TcTST_exists(c1, c2))
  {
    length[hc_gc_rs::TcTST] =
        gc00_reeds_shepp_->TcTST_path(c1, c2, &cstart[hc_gc_rs::TcTST], &cend[hc_gc_rs::TcTST], &qi1[hc_gc_rs::TcTST],
                                      &qi2[hc_gc_rs::TcTST], &qi3[hc_gc_rs::TcTST], &ci1[hc_gc_rs::TcTST]);
  }
  // case TcTSTcT
  if (gc00_reeds_shepp_->TcTSTcT_exists(c1, c2))// C|CSC|C
  {
    length[hc_gc_rs::TcTSTcT] = gc00_reeds_shepp_->TcTSTcT_path(
        c1, c2, &cstart[hc_gc_rs::TcTSTcT], &cend[hc_gc_rs::TcTSTcT], &qi1[hc_gc_rs::TcTSTcT], &qi2[hc_gc_rs::TcTSTcT],
        &qi3[hc_gc_rs::TcTSTcT], &qi4[hc_gc_rs::TcTSTcT], &ci1[hc_gc_rs::TcTSTcT], &ci2[hc_gc_rs::TcTSTcT]);
  }
  // case TTcTT
  if (gc00_reeds_shepp_->TTcTT_exists(c1, c2))
  {
    length[hc_gc_rs::TTcTT] = gc00_reeds_shepp_->TTcTT_path(
        c1, c2, &cstart[hc_gc_rs::TTcTT], &cend[hc_gc_rs::TTcTT], &qi1[hc_gc_rs::TTcTT], &qi2[hc_gc_rs::TTcTT],
        &qi3[hc_gc_rs::TTcTT], &ci1[hc_gc_rs::TTcTT], &ci2[hc_gc_rs::TTcTT]);
  }
  // case TcTTcT
  if (gc00_reeds_shepp_->TcTTcT_exists(c1, c2))
  {
    length[hc_gc_rs::TcTTcT] = gc00_reeds_shepp_->TcTTcT_path(
        c1, c2, &cstart[hc_gc_rs::TcTTcT], &cend[hc_gc_rs::TcTTcT], &qi1[hc_gc_rs::TcTTcT], &qi2[hc_gc_rs::TcTTcT],
        &qi3[hc_gc_rs::TcTTcT], &ci1[hc_gc_rs::TcTTcT], &ci2[hc_gc_rs::TcTTcT]);
  }
  // ############################# Additional case for HC family
  // case TTT
  if (gc00_reeds_shepp_->TTT_exists(c1, c2))
  {
    length[hc_gc_rs::TTT] = gc00_reeds_shepp_->TTT_path(c1, c2, &cstart[hc_gc_rs::TTT], &cend[hc_gc_rs::TTT],
                                                        &qi1[hc_gc_rs::TTT], &qi2[hc_gc_rs::TTT], &ci1[hc_gc_rs::TTT]);
  }
  // case TcST
  if (gc00_reeds_shepp_->TcST_exists(c1, c2))
  {// (4.0, 8.0, -1.5pi)
    length[hc_gc_rs::TcST] = gc00_reeds_shepp_->TcST_path(c1, c2, &cstart[hc_gc_rs::TcST], &cend[hc_gc_rs::TcST],
                                                          &qi1[hc_gc_rs::TcST], &qi2[hc_gc_rs::TcST]);
  }
  // case TScT
  if (gc00_reeds_shepp_->TScT_exists(c1, c2))
  {// (8, -4, 0.5pi) 어쩔 때는 되는데.... 왜그러지... 되는 경우... (10, -4, -0.5pi)
    length[hc_gc_rs::TScT] = gc00_reeds_shepp_->TScT_path(c1, c2, &cstart[hc_gc_rs::TScT], &cend[hc_gc_rs::TScT],
                                                          &qi1[hc_gc_rs::TScT], &qi2[hc_gc_rs::TScT]);
  }
  // case TcScT
  if (gc00_reeds_shepp_->TcScT_exists(c1, c2))
  {
    length[hc_gc_rs::TcScT] = gc00_reeds_shepp_->TcScT_path(c1, c2, &cstart[hc_gc_rs::TcScT], &cend[hc_gc_rs::TcScT],
                                                            &qi1[hc_gc_rs::TcScT], &qi2[hc_gc_rs::TcScT]);
  }
label_end:
  // select shortest path
  hc_gc_rs::path_type best_path = (hc_gc_rs::path_type)array_index_min(length, number_of_candidate_path);
  GC_Path *path;
  path = new GC_Path(c1.start, c2.start, best_path, kappa_, sigma_sc_, sigma_cs_, qi1[best_path], qi2[best_path],
                           qi3[best_path], qi4[best_path], cstart[best_path], cend[best_path], ci1[best_path],
                           ci2[best_path], length[best_path]);

  // clean up
  for (int i = 0; i < number_of_candidate_path; i++)
  {
    if (i != best_path)
    {
      delete qi1[i];
      delete qi2[i];
      delete qi3[i];
      delete qi4[i];
      delete cstart[i];
      delete ci1[i];
      delete ci2[i];
      delete cend[i];
    }
  }
  return path;
}

GC_Path *GC00_Reeds_Shepp_State_Space::gc00_reeds_shepp(const hcState &state1, const hcState &state2) const
{
  // compute the 4 circles at the intial and final configuration (state1 == start_state, state2 == goal_state)
  Configuration start(state1.x, state1.y, state1.theta, 0.0);
  Configuration end(state2.x, state2.y, state2.theta, 0.0);
  // @MSK: 시작 및 도착 위치에 각각 원을 4개 생성.//array
  GC_Circle *start_circle[4];
  GC_Circle *end_circle[4];
  // GC_Circle input is [_start, _left, _forward, _regular, straight_to_circle]
  start_circle[0] = new GC_Circle(start, true, true, true, true, gc_circle_param_);// 왼쪽 앞
  start_circle[1] = new GC_Circle(start, false, true, true, true, gc_circle_param_);// 오른쪽 앞
  start_circle[2] = new GC_Circle(start, true, false, true, true, gc_circle_param_);// 왼쪽 뒤
  start_circle[3] = new GC_Circle(start, false, false, true, true, gc_circle_param_);// 오른쪽 뒤
  end_circle[0] = new GC_Circle(end, true, true, true, false, gc_circle_param_);// GOAL = q_g
  end_circle[1] = new GC_Circle(end, false, true, true, false, gc_circle_param_);
  end_circle[2] = new GC_Circle(end, true, false, true, false, gc_circle_param_);
  end_circle[3] = new GC_Circle(end, false, false, true, false, gc_circle_param_);

  // compute the shortest path for the 16 combinations (4 circles at the beginning and 4 at the end)
  GC_Path *path[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

  // @MSK: lg는 각 path에 대응되는 길이를 저장.// 즉, q_s와 q_g의 총 16가지 조합 경우 모두 생성 후, '길이로' 어떤 path 생성할 지 비교.
  double lg[] = { numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), 
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), 
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max() };

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      path[4 * i + j] = gc00_circles_rs_path(*start_circle[i], *end_circle[j]);
      if (path[4 * i + j])
      {
        lg[4 * i + j] = path[4 * i + j]->length;
      }
    }
  }
  // select shortest path
  int best_path = array_index_min(lg, 16);

  // clean up// @MSK: for memory?
  for (int i = 0; i < 4; i++)
  {
    delete start_circle[i];
    delete end_circle[i];
  }
  for (int i = 0; i < 16; i++)
  {
    if (i != best_path)
    {
      delete path[i];
    }
  }
  return path[best_path];
}

double GC00_Reeds_Shepp_State_Space::get_distance(const hcState &state1, const hcState &state2) const
{
  GC_Path *p = this->gc00_reeds_shepp(state1, state2);
  double length = p->length;
  delete p;
  return length;
}

pair<vector<GC_Circle>, vector<Configuration>> GC00_Reeds_Shepp_State_Space::get_path_configuration(const hcState &state1, const hcState &state2)
{
  GC_Path *p = this->gc00_reeds_shepp(state1, state2);//state1 = start, state2 = goal
  vector<GC_Circle> gc_circle_arrays;
  vector<Configuration> q_arrays;
  if (p->qi1)
    q_arrays.push_back(*(p->qi1));
  if (p->qi2)
    q_arrays.push_back(*(p->qi2));
  if (p->qi3)
    q_arrays.push_back(*(p->qi3));
  if (p->qi4)
    q_arrays.push_back(*(p->qi4));
  
  if (p->cstart) 
    gc_circle_arrays.push_back(*(p->cstart));
  if (p->ci1) 
    gc_circle_arrays.push_back(*(p->ci1));
  if (p->ci2) 
    gc_circle_arrays.push_back(*(p->ci2));
  if (p->cend) 
    gc_circle_arrays.push_back(*(p->cend));
  
  return std::make_pair(gc_circle_arrays, q_arrays);
}

vector<Control> GC00_Reeds_Shepp_State_Space::get_controls(const hcState &state1, const hcState &state2) const
{
  vector<Control> gc_rs_controls;
  //@MSK: Control 구조체는 {delta_s, kappa, sigma}로 구성.
  gc_rs_controls.reserve(10);// vector 공간 확보
  GC_Path *p = this->gc00_reeds_shepp(state1, state2);//state1 = start, state2 = goal

  switch (p->type)
  {
    case hc_gc_rs::E:
      empty_controls(gc_rs_controls);
      break;
    case hc_gc_rs::S:
      straight_controls(p->start, p->end, gc_rs_controls);
      break;
    case hc_gc_rs::T:
      gc_turn_controls(*(p->cstart), p->end, true, gc_rs_controls);
      break;
    case hc_gc_rs::TT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi1), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcT:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi1), false, gc_rs_controls);
      break;
    // ##### Reeds-Shepp families: ############################################
    case hc_gc_rs::TcTcT:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_rs_turn_controls(*(p->ci1), *(p->qi2), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcTT:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi1), false, gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TTcT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi2), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TST:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      straight_controls(*(p->qi1), *(p->qi2), gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TSTcT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      straight_controls(*(p->qi1), *(p->qi2), gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi3), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi3), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcTST:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi1), false, gc_rs_controls);
      straight_controls(*(p->qi2), *(p->qi3), gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi3), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcTSTcT:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi1), false, gc_rs_controls);
      straight_controls(*(p->qi2), *(p->qi3), gc_rs_controls);
      gc_hc_turn_controls(*(p->ci2), *(p->qi4), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi4), false, gc_rs_controls);
      break;
    case hc_gc_rs::TTcTT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci1), *(p->qi2), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci2), *(p->qi2), false, gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi3), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcTTcT:
      //@MSK: TcTTcT의 각 의미는 결국: Turn-Change-Turn-Turn-Change-Turn. 즉, C|CC|C 이며 RS familiy 중 한 녀석.
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);//(circle, configuration, order, controls)
      gc_hc_turn_controls(*(p->ci1), *(p->qi1), false, gc_rs_controls);
      gc_hc_turn_controls(*(p->ci2), *(p->qi3), true, gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi3), false, gc_rs_controls);
      break;
    // ########################################################################
    case hc_gc_rs::TTT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      gc_turn_controls(*(p->ci1), *(p->qi2), true, gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcST:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      straight_controls(*(p->qi1), *(p->qi2), gc_rs_controls);
      gc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TScT:
      gc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      straight_controls(*(p->qi1), *(p->qi2), gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    case hc_gc_rs::TcScT:
      gc_hc_turn_controls(*(p->cstart), *(p->qi1), true, gc_rs_controls);
      straight_controls(*(p->qi1), *(p->qi2), gc_rs_controls);
      gc_hc_turn_controls(*(p->cend), *(p->qi2), false, gc_rs_controls);
      break;
    default:
      break;
  }
  delete p;
  return gc_rs_controls;
}
