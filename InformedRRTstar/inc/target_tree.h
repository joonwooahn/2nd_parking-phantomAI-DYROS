#ifndef TARGET_TREE_GENERATOR
#define TARGET_TREE_GENERATOR

#include "./utilities/utilities.h"
#include "./utilities/steering_functions.h"

#include <limits>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include <deque>
#include <utility>
#include <list>
#include <fstream>
#include <string.h>

#define DIM 10.0
using namespace Eigen;

 /** \brief Representation of a motion */
namespace tree
{
    class Node
    {
    public:
        /** \brief Constructor that allocates memory for the state. This constructor automatically allocates **/
        Node() : parent(nullptr)
        {
        }

        ~Node() = default;

        /** \brief The state contained by the motion */
        steer::hcState *state;

        /** \brief The parent motion in the exploration tree */
        Node *parent;

        /** \brief The type of this node belongs, ex: 0: straight line, 1: clothoid, 2: circular arc  */
        string which_type;

        /** \brief The sharpness of this node. If it is on the straight_line, this value is zero */
        double curvature_derivative;

        /** \brief The cost up to this motion */
        double distance_to_goal;

        /** \brief The set of motions descending from the current motion */
        std::vector<Node *> children;

        void print() {
            std::cout << std::endl;
            if (parent != nullptr) std::cout << "has parent node" << std::endl;
            std::cout << "type: " << which_type << std::endl;
            std::cout << "curvature_derivative: " << curvature_derivative << std::endl;
            std::cout << "distance to goal: " << distance_to_goal << std::endl;
            std::cout << "How many children? " << children.size() << std::endl;
        }
    };
}

/** \brief Target Tree generator */
class target_tree_generator// it generates goal-reachable node
{
public:
    typedef std::function<bool(float x, float y, double yaw)> isFreeSpace;

    target_tree_generator(const double max_curvature, const double max_curvature_derivative, const isFreeSpace &valid_checker)
    : max_curvature_(max_curvature), min_turning_radius_(1.0 / max_curvature), max_curvature_derivative_(max_curvature_derivative), is_free_space_(valid_checker)
    {
    }

    ~target_tree_generator()
    {
    }

    double max_curvature_;
    double min_turning_radius_;
    double max_curvature_derivative_;
    double interval_ = 0.1;
    double score_ =  999.9;
    isFreeSpace is_free_space_;

    vector<tree::Node *> target_tree_;

    void set_interval(double interval) {
        interval_ = interval;
    }

    void clear_tree() {
        // for(int i = 0; i < m_tree_idx; i++)
        target_tree_.clear();
        // m_tree_idx = 0;
    }

    double polygonArea(std::vector<std::pair<double, double>> point_set) {
        // Initialze area
        double area = 0.0;
        int n = point_set.size();
        // Calculate value of shoelace formula
        int j = n - 1;
        for (int i = 0; i < n; i++)
        {
            area += (point_set[j].first + point_set[i].first) * (point_set[j].second - point_set[i].second);
            j = i;  // j is previous vertex to i
        }

        // Return absolute value
        return abs(area / 2.0);
    }

    void build_target_tree(double x_s, double y_s, double theta_s, double straight_length, double turn_length, int num_branches) {
        vector<double> sigma_set = linspace(-max_curvature_derivative_, max_curvature_derivative_, num_branches, true);
        // std::vector<double> sigma_set{-0.250, -0.13, -0.09, -0.065, -0.05, -0.04, -0.030, -0.02, -0.01, 0.0, 0.01, 0.02, 0.030, 0.04, 0.05, 0.065, 0.09, 0.13, 0.25};// for visualize
        // std::vector<double> sigma_set{-0.145, -0.10, -0.07, -0.05, -0.045, -0.030, -0.02, -0.01, 0.0, 0.01, 0.02, 0.030, 0.045, 0.05, 0.07, 0.10, 0.145};

        double max_rx{0.0}, max_ry{0.0};
        double max_lx{0.0}, max_ly{0.0};
        
        // for cost function
        double temp_xi, temp_yi, temp_thi, temp_kappai;
        end_of_clothoid(0.0, 0.0, 0.0, 0.0, max_curvature_derivative_, 1, max_curvature_ / max_curvature_derivative_, &temp_xi, &temp_yi, &temp_thi, &temp_kappai);
        double dy_cl = temp_yi;
        double cir_dth = (turn_length - max_curvature_ / max_curvature_derivative_) / min_turning_radius_;
        double clo_dth = 0.5 * pow(max_curvature_, 2) / max_curvature_derivative_;
        double max_x = turn_length;
        double max_y = dy_cl + 2.0 * min_turning_radius_ * sin(cir_dth / 2.0) * sin(cir_dth / 2.0 + clo_dth);

        double clothoid_length;
        double x_i, y_i, theta_i, kappa_i;
        double clo_x_i, clo_y_i, clo_th_i, clo_kappa_i;
        double cir_x_i, cir_y_i, cir_th_i, cir_kappa_i;
        bool collision_detect = false;
        int prev_ind, end_of_straight_line_ind;

        x_i = x_s;  y_i = y_s;  theta_i = theta_s;  kappa_i = 0.0;

        // generate straight line
        for (int i = 0; i * interval_ <= straight_length; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            target_state->x = x_i = x_i + interval_ * cos(theta_s);
            target_state->y = y_i = y_i + interval_ * sin(theta_s);
            target_state->theta = theta_s;
            target_state->kappa = 0.0; target_state->d = 0.0;

            if (!is_free_space_(x_i, y_i, theta_i) || fabs(x_i) > DIM || fabs(y_i) > DIM) {
                collision_detect = true;
                break;
            }
            if (i != 0) {//at first
                node->parent = target_tree_[i-1];
                node->parent->children.push_back(node);
            }
            node->state = target_state;
            node->which_type = "straight line";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = i * interval_;
            target_tree_.push_back(node);
        }// at the end, x_i and y_i are states of the end of straight line.
        end_of_straight_line_ind = target_tree_.size() - 1;
        prev_ind = target_tree_.size() - 1;// store the index of the last straight line

        if (collision_detect) { // it means, in the straight line, the collision occurs, and it is not going to be resolved.
            return;
        }

        for (const auto &sigma : sigma_set) {
            collision_detect = false;
            if (sigma != 0.0) {
                clothoid_length = min(max_curvature_ / fabs(sigma), turn_length);//@MSK l_min in the paper
                // generate a clothoid arc
                for (int i = 0; (i+1) * interval_ <= clothoid_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_clothoid(x_i, y_i, theta_i, 0.0, sigma, 1, (i+1) * interval_, &clo_x_i, &clo_y_i, &clo_th_i, &clo_kappa_i);
                    target_state->x = clo_x_i;   target_state->y = clo_y_i;   target_state->theta = clo_th_i;  target_state->kappa = clo_kappa_i;
                    if (collision_detect || !is_free_space_(clo_x_i, clo_y_i, clo_th_i) || fabs(clo_x_i) > DIM || fabs(clo_y_i) > DIM) {
                        collision_detect = true;
                        if (sigma < 0.0) {
                            double dth = atan2(clo_y_i - y_i, clo_x_i - x_i) - theta_s;
                            double dL = sqrt(pow(clo_x_i - x_i, 2) + pow(clo_y_i - y_i, 2));
                            max_rx = max(fabs(dL * cos(dth)), max_rx);
                            max_ry = max(fabs(dL * sin(dth)), max_ry);
                        }
                        else if (sigma > 0.0) {
                            double dth = atan2(clo_y_i - y_i, clo_x_i - x_i) - theta_s;
                            double dL = sqrt(pow(clo_x_i - x_i, 2) + pow(clo_y_i - y_i, 2));
                            max_lx = max(fabs(dL * cos(dth)), max_lx);
                            max_ly = max(fabs(dL * sin(dth)), max_ly);
                        }
                        break;
                    }

                    if (i == 0) {
                        node->parent = target_tree_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_tree_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "clothoid arc";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = straight_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }
                end_of_clothoid(x_i, y_i, theta_i, 0.0, sigma, 1, clothoid_length, &clo_x_i, &clo_y_i, &clo_th_i, &clo_kappa_i);

                // generate a circular arc
                for (int i = 0; !collision_detect && (i+1) * interval_ <= turn_length - clothoid_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_circular_arc(clo_x_i, clo_y_i, clo_th_i, clo_kappa_i, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);
                    target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = -clo_kappa_i;
                    if (collision_detect || !is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        if (sigma < 0.0) {
                            double dth = atan2(cir_y_i - y_i, cir_x_i - x_i) - theta_s;
                            double dL = sqrt(pow(cir_x_i - x_i, 2) + pow(cir_y_i - y_i, 2));
                            max_rx = max(fabs(dL * cos(dth)), max_rx);
                            max_ry = max(fabs(dL * sin(dth)), max_ry);
                        }
                        else if (sigma > 0.0) {
                            double dth = atan2(cir_y_i - y_i, cir_x_i - x_i) - theta_s;
                            double dL = sqrt(pow(cir_x_i - x_i, 2) + pow(cir_y_i - y_i, 2));
                            max_lx = max(fabs(dL * cos(dth)), max_lx);
                            max_ly = max(fabs(dL * sin(dth)), max_ly);
                        }
                        break;
                    }

                    node->parent = target_tree_[prev_ind];// it recursively stores a parent
                    node->parent->children.push_back(node);
                    node->state = target_state;
                    node->which_type = "circular arc";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = straight_length + clothoid_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }

                if (!collision_detect) {// path is found fully
                    if (sigma < 0.0) {
                        double dth = atan2(cir_y_i - y_i, cir_x_i - x_i) - theta_s;
                        double dL = sqrt(pow(cir_x_i - x_i, 2) + pow(cir_y_i - y_i, 2));
                        max_rx = max(fabs(dL * cos(dth)), max_rx);
                        max_ry = max(fabs(dL * sin(dth)), max_ry);
                    }
                    else if (sigma > 0.0) {
                        double dth = atan2(cir_y_i - y_i, cir_x_i - x_i) - theta_s;
                        double dL = sqrt(pow(cir_x_i - x_i, 2) + pow(cir_y_i - y_i, 2));
                        max_lx = max(fabs(dL * cos(dth)), max_lx);
                        max_ly = max(fabs(dL * sin(dth)), max_ly);
                    }
                }
            }
            else {
                // When the sigma is zero, which means it is the straight line.
                double last_x = 0.0, last_y = 0.0;
                double last_length = 0.0;
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    target_state->x = target_tree_[end_of_straight_line_ind]->state->x + (i+1) * interval_ * cos(theta_s);
                    target_state->y = target_tree_[end_of_straight_line_ind]->state->y + (i+1) * interval_ * sin(theta_s);
                    target_state->theta = theta_s;
                    target_state->kappa = 0.0; target_state->d = 0.0;

                    if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }
                    last_length = i * interval_;
                    last_x = target_state->x; last_y = target_state->y;

                    if (i == 0) {
                        node->parent = target_tree_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_tree_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "straight line";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = straight_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }
                
                double dL = sqrt(pow(last_x - x_i, 2) + pow(last_y - y_i, 2));
                max_rx = max(dL, max_rx);
                max_lx = max(dL, max_lx);
                // max_rx = max(last_x - straight_length, max_rx);
                // max_lx = max(last_x - straight_length, max_lx);
            }
        }
       
        score_ = (max_rx * max_ry + max_lx * max_ly) / (2.0 * max_x * max_y);
        cout << "COST: " << 1.0 - score_ << endl;
    };


    void build_target_tree_parallel(double x_s, double y_s, double theta_s, double straight_length, double turn_length, int num_branches) {
        vector<double> sigma_set = linspace(-max_curvature_derivative_, max_curvature_derivative_, num_branches, true);
        // std::vector<double> sigma_set{-0.145, -0.07, -0.045, -0.030, -0.02, -0.01, 0.0, 0.01, 0.02, 0.030, 0.045, 0.07, 0.145};
        // std::vector<double> sigma_set{-0.200, -0.13, -0.09, -0.065, -0.05, -0.04, -0.030, -0.02, -0.01, 0.0, 0.01, 0.02, 0.030, 0.04, 0.05, 0.065, 0.09, 0.13, 0.20};

        // for cost function
        double temp_xi, temp_yi, temp_thi, temp_kappai;
        end_of_clothoid(0.0, 0.0, 0.0, 0.0, max_curvature_derivative_, 1, max_curvature_ / max_curvature_derivative_, &temp_xi, &temp_yi, &temp_thi, &temp_kappai);
        double max_rx{0.0}, max_ry{0.0};
        double max_lx{0.0}, max_ly{0.0};
        double end_bx, end_by, end_bth; // end of the branch coordinates

        double dy_cl = temp_yi;
        double cir_dth = (turn_length - max_curvature_ / max_curvature_derivative_) / min_turning_radius_;
        double clo_dth = 0.5 * pow(max_curvature_, 2) / max_curvature_derivative_;
        double max_x = turn_length;
        double max_y = dy_cl + 2.0 * min_turning_radius_ * sin(cir_dth / 2.0) * sin(cir_dth / 2.0 + clo_dth);

        double clothoid_length;
        double x_i, y_i, theta_i, kappa_i;
        double clo_x_i, clo_y_i, clo_th_i, clo_kappa_i;
        double cir_x_i, cir_y_i, cir_th_i, cir_kappa_i;
        bool collision_detect = false;
        int prev_ind, end_of_straight_line_ind;
        double escapte_path_length{0.0};

        
        // Insert the goal
        x_i = x_s;  y_i = y_s;  theta_i = theta_s;  kappa_i = 0.0;
        auto *node = new tree::Node();
        auto *target_state = new steer::hcState;
        target_state->x = x_i;   target_state->y = y_i;   target_state->theta = theta_i;  target_state->kappa = kappa_i;
        node->state = target_state;
        node->which_type = "goal";
        node->curvature_derivative = 0.0;
        node->distance_to_goal = 0.0;
        target_tree_.push_back(node);
        prev_ind = target_tree_.size() - 1;// store the previous index
        cout << "GOAL is Inserted" << endl;
        // How to define a set of circular arcs.
        // generate a backward circular path
        
        int recent_ind;
        int num_of_set = 2;
        for (int k = 0; k < num_of_set; k++) {
            for (int i = 0; (i+1) * interval_ <= 2.0; i++)  {
                auto *node = new tree::Node();
                auto *target_state = new steer::hcState;
                end_of_circular_arc(x_i, y_i, theta_i, -max_curvature_, -1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);  // if it is in right with respect to the vehicle, {-max_curv, -1}
                target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = -max_curvature_;

                if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                    break;
                }

                node->parent = target_tree_[prev_ind];
                node->parent->children.push_back(node);
                node->state = target_state;
                node->which_type = "pre-circular path (backward)";
                node->curvature_derivative = 0.0;
                node->distance_to_goal = (i+1) * interval_;
                target_tree_.push_back(node);
                prev_ind = target_tree_.size() - 1;// store the previous index
                escapte_path_length += interval_;
                recent_ind = i;
            }
            end_of_circular_arc(x_i, y_i, theta_i, -max_curvature_, -1, recent_ind * interval_, &cir_x_i, &cir_y_i, &cir_th_i);
            x_i = cir_x_i; y_i = cir_y_i; theta_i = cir_th_i; kappa_i = -max_curvature_;

            // generate a forward circular path
            for (int i = 0; (i+1) * interval_ <= 1.0; i++)  {
                auto *node = new tree::Node();
                auto *target_state = new steer::hcState;
                end_of_circular_arc(x_i, y_i, theta_i, max_curvature_, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i); // 원형 경로는 전진으로...
                target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = max_curvature_;

                if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                    break;
                }
                node->parent = target_tree_[prev_ind];
                node->parent->children.push_back(node);
                node->state = target_state;
                node->which_type = "pre-circular path (forward))";
                node->curvature_derivative = 0.0;
                node->distance_to_goal = target_tree_.back()->distance_to_goal + (i+1) * interval_;
                target_tree_.push_back(node);
                prev_ind = target_tree_.size() - 1;// store the previous index
                escapte_path_length += interval_;
            }
            x_i = cir_x_i; y_i = cir_y_i; theta_i = cir_th_i; kappa_i = max_curvature_;
        }
        cout << "reorientation is finishted" << endl;

        // generate a forward clothoid path
        for (int i = 0; (i+1) * interval_ < fabs(max_curvature_ / max_curvature_derivative_) ; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            end_of_clothoid(x_i, y_i, theta_i, max_curvature_, -max_curvature_derivative_, 1, (i+1) * interval_, &clo_x_i, &clo_y_i, &clo_th_i, &clo_kappa_i);
            target_state->x = clo_x_i;   target_state->y = clo_y_i;   target_state->theta = clo_th_i;  target_state->kappa = clo_kappa_i;
            if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                collision_detect = true;
                break;
            }

            node->parent = target_tree_[prev_ind];// it recursively stores a parent
            node->parent->children.push_back(node);
            node->state = target_state;
            node->which_type = "pre-clothoid arc (forward)";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = target_tree_.back()->distance_to_goal + (i+1) * interval_;
            target_tree_.push_back(node);
            prev_ind = target_tree_.size() - 1;// store the previous index
            escapte_path_length += interval_;
        }
        x_i = clo_x_i;  y_i = clo_y_i;  theta_i = clo_th_i;  kappa_i = 0.0;

        // generate straight line.
        for (int i = 1; i * interval_ < straight_length; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            target_state->x = x_i = x_i + interval_ * cos(theta_i);
            target_state->y = y_i = y_i + interval_ * sin(theta_i);
            target_state->theta = theta_i;
            target_state->kappa = 0.0; target_state->d = 0.0;

            if (!is_free_space_(x_i, y_i, theta_i) || fabs(x_i) > DIM || fabs(y_i) > DIM) {
                collision_detect = true;
                break;
            }
            node->parent = target_tree_[prev_ind];
            node->parent->children.push_back(node);
            node->state = target_state;
            node->which_type = "straight line";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = escapte_path_length + i * interval_;
            target_tree_.push_back(node);
            prev_ind = target_tree_.size() - 1;// store the index of the last index
        }// at the end, x_i and y_i are states of the end of straight line.
        end_of_straight_line_ind = target_tree_.size() - 1;
        end_bx = x_i;
        end_by = y_i;
        end_bth = theta_i;

        for (const auto &sigma : sigma_set) {
            collision_detect = false;
            if (sigma != 0.0) {
                clothoid_length = min(max_curvature_ / fabs(sigma), turn_length);//@MSK l_min in the paper
                // generate a clothoid arc
                for (int i = 0; (i+1) * interval_ <= clothoid_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_clothoid(x_i, y_i, theta_i, 0.0, sigma, 1, (i+1) * interval_, &clo_x_i, &clo_y_i, &clo_th_i, &clo_kappa_i);
                    target_state->x = clo_x_i;   target_state->y = clo_y_i;   target_state->theta = clo_th_i;  target_state->kappa = clo_kappa_i;
                    if ((collision_detect || !is_free_space_(clo_x_i, clo_y_i, clo_th_i) || fabs(clo_x_i) > DIM || fabs(clo_y_i) > DIM)) {
                        collision_detect = true;
                        break;
                    }
                    // ========== not collision detect
                    end_bx = target_state->x;
                    end_by = target_state->y;
                    end_bth = target_state->theta;

                    if (i == 0) {
                        node->parent = target_tree_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_tree_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "clothoid arc";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = escapte_path_length + straight_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }
                end_of_clothoid(x_i, y_i, theta_i, 0.0, sigma, 1, clothoid_length, &clo_x_i, &clo_y_i, &clo_th_i, &clo_kappa_i);

                // generate a circular arc
                for (int i = 0; !collision_detect && (i+1) * interval_ <= turn_length - clothoid_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_circular_arc(clo_x_i, clo_y_i, clo_th_i, clo_kappa_i, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);
                    target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = -clo_kappa_i;
                    if ((collision_detect || !is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM)) {
                        collision_detect = true;
                        break;
                    }
                    // ========== not collision detect
                    end_bx = target_state->x;
                    end_by = target_state->y;
                    end_bth = target_state->theta;

                    node->parent = target_tree_[prev_ind];// it recursively stores a parent
                    node->parent->children.push_back(node);
                    node->state = target_state;
                    node->which_type = "circular arc";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = escapte_path_length + straight_length + clothoid_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }
                double dL = sqrt(pow(end_bx - x_i, 2) + pow(end_by - y_i, 2));
                double dth = atan2(end_by - y_i, end_bx - x_i);

                if (sigma < 0.0) {
                    max_rx = max(max_rx, dL * fabs(cos(dth - theta_i)));
                    max_ry = max(max_ry, dL * fabs(sin(dth - theta_i)));
                }
                else  {// sigma > 0.0
                    max_lx = max(max_lx, dL * fabs(cos(dth - theta_i)));
                    max_ly = max(max_ly, dL * fabs(sin(dth - theta_i)));
                }

            }
            else {
                // When the sigma is zero, which means it is the straight line.
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    // cout << "i: " << i << endl;
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    target_state->x = target_tree_[end_of_straight_line_ind]->state->x + (i+1) * interval_ * cos(target_tree_[end_of_straight_line_ind]->state->theta);
                    target_state->y = target_tree_[end_of_straight_line_ind]->state->y + (i+1) * interval_ * sin(target_tree_[end_of_straight_line_ind]->state->theta);
                    target_state->theta = target_tree_[end_of_straight_line_ind]->state->theta;
                    target_state->kappa = 0.0; target_state->d = 0.0;
                    if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }
                    // ========== not collision detect
                    end_bx = target_state->x;
                    end_by = target_state->y;

                    if (i == 0) {
                        node->parent = target_tree_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_tree_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "straight line";
                    node->curvature_derivative = sigma;
                    node->distance_to_goal = escapte_path_length + straight_length + (i+1) * interval_;
                    target_tree_.push_back(node);
                    prev_ind = target_tree_.size() - 1;
                }
                double dL = sqrt(pow(end_bx - x_i, 2) + pow(end_by - y_i, 2));
                max_rx = max(dL, max_rx);
                max_lx = max(dL, max_lx);
            }

        }
        std::cout << std::endl;
        std::cout << "Tree size: " << target_tree_.size() << std::endl;
        cout << "COST: " << 1.0 - (max_rx * max_ry + max_lx * max_ly) / (2.0 * 10.0 * 6.29984) << endl;
        score_ = (max_rx * max_ry + max_lx * max_ly);
    };

};

/** \brief Target Tree generator */
class target_point_generator// it generates goal-reachable node
{
public:
    typedef std::function<bool(float x, float y, double yaw)> isFreeSpace;

    target_point_generator(const double max_curvature, const isFreeSpace &valid_checker)
    : max_curvature_(max_curvature), min_turning_radius_(1.0 / max_curvature), is_free_space_(valid_checker)
    {
    }

    ~target_point_generator()
    {
    }

    double max_curvature_;
    double min_turning_radius_;
    double interval_ = 0.01;
    isFreeSpace is_free_space_;

    vector<tree::Node *> target_point_;

    void clear_tree() {
        target_point_.clear();
    }

    void build_target_point(double x_s, double y_s, double theta_s, double straight_length, double turn_length, int num_branches) {
        vector<double> curvature_set = linspace(-max_curvature_, max_curvature_, num_branches, true);
        double x_i, y_i, theta_i, kappa_i;
        double cir_x_i, cir_y_i, cir_th_i, cir_kappa_i;
        bool collision_detect = false;
        int prev_ind, end_of_straight_line_ind;

        x_i = x_s;  y_i = y_s;  theta_i = theta_s;  kappa_i = 0.0;

        // generate straight line
        for (int i = 0; i * interval_ <= straight_length; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            target_state->x = x_i = x_i + interval_ * cos(theta_s);
            target_state->y = y_i = y_i + interval_ * sin(theta_s);
            target_state->theta = theta_s;
            target_state->kappa = 0.0; target_state->d = 0.0;

            if (!is_free_space_(x_i, y_i, theta_i) || fabs(x_i) > DIM || fabs(y_i) > DIM) {
                collision_detect = true;
                break;
            }
            if (i != 0) {//at first
                node->parent = target_point_[i-1];
                node->parent->children.push_back(node);
            }
            node->state = target_state;
            node->which_type = "straight line";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = i * interval_;
            target_point_.push_back(node);
        }// at the end, x_i and y_i are states of the end of straight line.
        end_of_straight_line_ind = target_point_.size() - 1;
        prev_ind = target_point_.size() - 1;// store the index of the last straight line

        if (collision_detect) { // it means, in the straight line, the collision occurs, and it is not going to be resolved.
            return;
        }
        for (const auto &curv_ : curvature_set) {
            collision_detect = false;
            if (curv_ != 0.0) {
                // generate a circular arc
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_circular_arc(x_i, y_i, theta_i, curv_, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);

                    target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = curv_;
                    if (collision_detect || !is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }

                    if (i == 0) {
                        node->parent = target_point_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_point_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "circular arc";
                    node->curvature_derivative = 0.0;
                    node->distance_to_goal = straight_length + (i+1) * interval_;
                    target_point_.push_back(node);
                    prev_ind = target_point_.size() - 1;
                }
                end_of_circular_arc(x_i, y_i, theta_i, curv_, 1, turn_length, &cir_x_i, &cir_y_i, &cir_th_i);

            }
            else {
                //curv is zero, which means straight line!
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    target_state->x = target_point_[end_of_straight_line_ind]->state->x + (i+1) * interval_ * cos(theta_s);
                    target_state->y = target_point_[end_of_straight_line_ind]->state->y + (i+1) * interval_ * sin(theta_s);
                    target_state->theta = theta_s;
                    target_state->kappa = 0.0; target_state->d = 0.0;
                    if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }
                    if (i == 0) {
                        node->parent = target_point_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_point_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "straight line";
                    node->curvature_derivative = 0.0;
                    node->distance_to_goal = straight_length + (i+1) * interval_;
                    target_point_.push_back(node);
                    prev_ind = target_point_.size() - 1;
                }
            }

        }
        std::cout << "Target point size: " << target_point_.size() << std::endl;
    };
    
    
    void build_target_point_parallel(double x_s, double y_s, double theta_s, double straight_length, double turn_length, int num_branches) {
        vector<double> curvature_set = linspace(-max_curvature_, max_curvature_, num_branches, true);
        double x_i, y_i, theta_i, kappa_i;
        double cir_x_i, cir_y_i, cir_th_i, cir_kappa_i;
        bool collision_detect = false;
        int prev_ind, end_of_straight_line_ind;
        double escapte_path_length;

        // Insert the goal
        x_i = x_s;  y_i = y_s;  theta_i = theta_s;  kappa_i = 0.0;
        auto *node = new tree::Node();
        auto *target_state = new steer::hcState;
        target_state->x = x_i;   target_state->y = y_i;   target_state->theta = theta_i;  target_state->kappa = kappa_i;
        node->state = target_state;
        node->which_type = "goal";
        node->curvature_derivative = 0.0;
        node->distance_to_goal = 0.0;
        target_point_.push_back(node);
        prev_ind = target_point_.size() - 1;// store the previous index

        // generate a backward straight path
        for (int i = 0; i * interval_ <= 2.0; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            target_state->x = x_i = x_i - interval_ * cos(theta_i);
            target_state->y = y_i = y_i - interval_ * sin(theta_i);
            target_state->theta = theta_i;
            target_state->kappa = 0.0;
            target_state->d = 0.0;
            if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                break;
            }

            node->parent = target_point_[prev_ind];
            node->parent->children.push_back(node);
            node->state = target_state;
            node->which_type = "pre-straight line (backward)";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = (i+1) * interval_;
            target_point_.push_back(node);
            prev_ind = target_point_.size() - 1;// store the previous index
            escapte_path_length += interval_;
        }
        x_i += interval_ * cos(theta_i);
        y_i += interval_ * sin(theta_i);

        // generate a forward circular path
        for (int i = 0; (i+1) * interval_ <= 3.0; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            end_of_circular_arc(x_i, y_i, theta_i, (6.0/5.0) * max_curvature_, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);
            target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = -max_curvature_;
            if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                break;
            }
            node->parent = target_point_[prev_ind];// it recursively stores a parent
            node->parent->children.push_back(node);
            node->state = target_state;
            node->which_type = "pre-cicular arc (forward)";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = target_point_.back()->distance_to_goal + (i+1) * interval_;
            target_point_.push_back(node);
            prev_ind = target_point_.size() - 1;// store the previous index
            escapte_path_length += interval_;
        }

        x_i = cir_x_i;
        y_i = cir_y_i;
        theta_i = cir_th_i;
        kappa_i = max_curvature_;//TODO

        // generate straight line
        for (int i = 1; i * interval_ <= straight_length; i++)  {
            auto *node = new tree::Node();
            auto *target_state = new steer::hcState;
            target_state->x = x_i = x_i + interval_ * cos(theta_i);
            target_state->y = y_i = y_i + interval_ * sin(theta_i);
            target_state->theta = theta_i;
            target_state->kappa = 0.0; target_state->d = 0.0;

            if (!is_free_space_(x_i, y_i, theta_i) || fabs(x_i) > DIM || fabs(y_i) > DIM) {
                collision_detect = true;
                break;
            }
            node->parent = target_point_[prev_ind];
            node->parent->children.push_back(node);
            node->state = target_state;
            node->which_type = "straight line";
            node->curvature_derivative = 0.0;
            node->distance_to_goal = escapte_path_length + i * interval_;
            target_point_.push_back(node);
            prev_ind = target_point_.size() - 1;// store the index of the last straight line
        }// at the end, x_i and y_i are states of the end of straight line.
        end_of_straight_line_ind = target_point_.size() - 1;

        for (const auto &curv_ : curvature_set) {
            collision_detect = false;
            if (curv_ != 0.0) {
                // generate a circular arc
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    end_of_circular_arc(x_i, y_i, theta_i, curv_, 1, (i+1) * interval_, &cir_x_i, &cir_y_i, &cir_th_i);
                    target_state->x = cir_x_i;   target_state->y = cir_y_i;   target_state->theta = cir_th_i;  target_state->kappa = curv_;
                    if (collision_detect || !is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }

                    if (i == 0) {
                        node->parent = target_point_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_point_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "circular arc";
                    node->curvature_derivative = 0.0;
                    node->distance_to_goal = escapte_path_length + straight_length + (i+1) * interval_;
                    target_point_.push_back(node);
                    prev_ind = target_point_.size() - 1;
                }
                end_of_circular_arc(x_i, y_i, theta_i, curv_, 1, turn_length, &cir_x_i, &cir_y_i, &cir_th_i);

            }
            else {
                //curv is zero, which means straight line!
                for (int i = 0; (i+1) * interval_ <= turn_length; i++)  {
                    auto *node = new tree::Node();
                    auto *target_state = new steer::hcState;
                    target_state->x = target_point_[end_of_straight_line_ind]->state->x + (i+1) * interval_ * cos(theta_i);
                    target_state->y = target_point_[end_of_straight_line_ind]->state->y + (i+1) * interval_ * sin(theta_i);
                    target_state->theta = theta_i;
                    target_state->kappa = 0.0; target_state->d = 0.0;
                    if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
                        collision_detect = true;
                        break;
                    }
                    if (i == 0) {
                        node->parent = target_point_[end_of_straight_line_ind];
                        node->parent->children.push_back(node);
                    }
                    else {
                        node->parent = target_point_[prev_ind];
                        node->parent->children.push_back(node);
                    }
                    node->state = target_state;
                    node->which_type = "straight line";
                    node->curvature_derivative = 0.0;
                    node->distance_to_goal = escapte_path_length + straight_length + (i+1) * interval_;
                    target_point_.push_back(node);
                    prev_ind = target_point_.size() - 1;
                }
            }

        }
        std::cout << "Target point size: " << target_point_.size() << std::endl;
    };


};
#endif

        // // generate a backward straight path
        // double x_src, y_src, theta_src;
        // for (int i = 0; i * interval_ <= 3.0; i++)  {
        //     auto *node = new tree::Node();
        //     auto *target_state = new steer::hcState;
        //     target_state->x = x_i = x_i - interval_ * cos(theta_i);
        //     target_state->y = y_i = y_i - interval_ * sin(theta_i);
        //     target_state->theta = theta_i;
        //     target_state->kappa = 0.0;
        //     target_state->d = 0.0;
        //     if (!is_free_space_(target_state->x, target_state->y, target_state->theta) || fabs(target_state->x) > DIM || fabs(target_state->y) > DIM) {
        //         break;
        //     }

        //     if (i != 0) {//at first
        //         node->parent = target_tree_[prev_ind];
        //         node->parent->children.push_back(node);
        //     }

        //     node->state = target_state;
        //     node->which_type = "pre-straight line (backward)";
        //     node->curvature_derivative = 0.0;
        //     node->distance_to_goal = (i+1) * interval_;
        //     target_tree_.push_back(node);
        //     prev_ind = target_tree_.size() - 1;// store the previous index
        //     escapte_path_length += interval_;
        // }
