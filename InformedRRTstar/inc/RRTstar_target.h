#ifndef OMPL_GEOMETRIC_PLANNERS_RRTSTAR_TARGET
#define OMPL_GEOMETRIC_PLANNERS_RRTSTAR_TARGET

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/MotionValidator.h>

#include "./utilities/steering_functions.h"
// #include "./utilities/hc00_reeds_shepp_state_space.h"
// #include "./reeds_shepp_state_space.h"

#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>
#include <fstream>
#include <ros/ros.h>
namespace ompl
{
    namespace geometric
    {
        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRTstar_target : public base::Planner
        {
        public:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates
                 * memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr), inGoal(false)
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The parent motion in the exploration tree */
                Motion *parent;

                /** \brief Set to true if this vertex is in the goal region */
                bool inGoal;

                /** \brief for target tree cost */
                double distance_to_goal{999.9};
                int ind;

                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
                 * computations in the updateChildCosts() method) */
                base::Cost incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;
            };


            typedef std::function<void(base::State *pose, bool random)> sampleView;
            typedef std::function<std::vector<double>(base::State *pose, double bias_ratio)> sampleBias;
            typedef std::function<void(base::State *pose, bool hc_tree)> add_node;
            typedef std::function<void(int ind)> remove_bias;
            typedef std::function<void(int node_ind)> best_update;
            typedef ompl::base::SE2StateSpace::StateType RSTATETYPE;

            std::vector<Motion *> rrt_tree;
            RRTstar_target(const base::SpaceInformationPtr &si,
                                const sampleBias &bias,
                                const remove_bias &rm_bias,
                                const best_update &best_update,
                                const double target_bias_ratio);

            ~RRTstar_target() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;
            void setup() override;

        private:
            bool using_hybrid_curvature_;
            sampleBias sb_;
            remove_bias rm_bias_;
            best_update best_update_;
            add_node add_node_to_tree_;
            double target_bias_ratio_;
            // base::SpaceInformationPtr rs_si_;
            // base::ReedsSheppMotionValidator *rs_mv;

        public:
            // void setLoggingPath();
            // std::ofstream logging_time;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            
            // void change_motion_to_node(Motion *motion, NODE *node)
            // {
            //     RSTATETYPE *motion_state = motion->state->as<RSTATETYPE>();
            //     node->x = motion_state->getX();
            //     node->y = motion_state->getY();
            //     node->yaw = motion_state->getYaw();
            //     if (motion->parent)
            //         node->having_parent = true;                
                

            // }

            void cchange_to_base_state(steer::hcState hc_s, base::State *base_state)
            {
                base_state->as<RSTATETYPE>()->setX(hc_s.x);
                base_state->as<RSTATETYPE>()->setY(hc_s.y);
                base_state->as<RSTATETYPE>()->setYaw(hc_s.theta);
            }

            bool satisfy_bound(steer::hcState hc_s, double bound)
            {
                if (hc_s.x > bound || hc_s.x < -bound ||
                    hc_s.y > bound || hc_s.y < -bound)
                    return false;
                return true;
            }


            steer::hcState cchange_to_steer_state(base::State *s)
            {
                steer::hcState steer_state;
                RSTATETYPE *pose_ = s->as<RSTATETYPE>();
                steer_state.x = pose_->getX();
                steer_state.y = pose_->getY();
                steer_state.theta = pose_->getYaw();
                steer_state.kappa = 0.0;
                steer_state.d = 0.0;
                return steer_state;    
            }

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*)
             */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
                calculateRewiringLowerBounds();
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times
             * k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collision checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Controls whether the tree is pruned during the search. This pruning removes
                a vertex if and only if it \e and all its descendents passes the pruning condition.
                The pruning condition is whether the lower-bounding estimate of a solution
                constrained to pass the the \e vertex is greater than the current solution.
                Considering the descendents of a vertex prevents removing a descendent
                that may actually be capable of later providing a better solution once
                its incoming path passes through a different vertex (e.g., a change in homotopy class). */
            void setTreePruning(bool prune);

            /** \brief Get the state of the pruning option. */
            bool getTreePruning() const
            {
                return useTreePruning_;
            }

            /** \brief Set the fractional change in solution cost necessary for pruning to occur, i.e.,
                prune if the new solution is at least X% better than the old solution.
                (e.g., 0.0 will prune after every new solution, while 1.0 will never prune.) */
            void setPruneThreshold(const double pp)
            {
                pruneThreshold_ = pp;
            }

            /** \brief Get the current prune states percentage threshold parameter. */
            double getPruneThreshold() const
            {
                return pruneThreshold_;
            }

            /** \brief Use the measure of the pruned subproblem instead of the measure of the entire problem domain (if
            such an expression exists and a solution is present).
            Currently the only method to calculate this measure in closed-form is through a informed sampler, so this
            option also requires that. */
            void setPrunedMeasure(bool informedMeasure);

            /** \brief Get the state of using the pruned measure */
            bool getPrunedMeasure() const
            {
                return usePrunedMeasure_;
            }

            /** \brief Use direct sampling of the heuristic for the generation of random samples (e.g., x_rand).
           If a direct sampling method is not defined for the objective, rejection sampling will be used by default. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state direct heuristic sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            /** \brief Controls whether heuristic rejection is used on samples (e.g., x_rand) */
            void setSampleRejection(bool reject);

            /** \brief Get the state of the sample rejection option */
            bool getSampleRejection() const
            {
                return useRejectionSampling_;
            }

            /** \brief Controls whether heuristic rejection is used on new states before connection (e.g., x_new =
             * steer(x_nearest, x_rand)) */
            void setNewStateRejection(const bool reject)
            {
                useNewStateRejection_ = reject;
            }

            /** \brief Get the state of the new-state rejection option */
            bool getNewStateRejection() const
            {
                return useNewStateRejection_;
            }

            /** \brief Controls whether pruning and new-state rejection uses an admissible cost-to-come estimate or not
             */
            void setAdmissibleCostToCome(const bool admissible)
            {
                useAdmissibleCostToCome_ = admissible;
            }

            /** \brief Get the admissibility of the pruning and new-state rejection heuristic */
            bool getAdmissibleCostToCome() const
            {
                return useAdmissibleCostToCome_;
            }

            /** \brief Controls whether samples are returned in ordered by the heuristic. This is accomplished by
             * generating a batch at a time. */
            void setOrderedSampling(bool orderSamples);

            /** \brief Get the state of sample ordering. */
            bool getOrderedSampling() const
            {
                return useOrderedSampling_;
            }

            /** \brief Set the batch size used for sample ordering*/
            void setBatchSize(unsigned int batchSize)
            {
                batchSize_ = batchSize;
            }

            /** \brief Get the batch size used for sample ordering*/
            unsigned int getBatchSize() const
            {
                return batchSize_;
            }

            /** \brief A \e meta parameter to focusing the search to improving the current solution. This is the
            parameter set by CFOREST.
            For RRT*, search focusing consists of pruning the existing search and limiting future search.
            Specifically, this is accomplished by turning on informed sampling, tree pruning and new-state rejection.
            This flag individually sets the options described above.
            */
            void setFocusSearch(const bool focus)
            {
                setInformedSampling(focus);
                setTreePruning(focus);
                setPrunedMeasure(focus);
                setNewStateRejection(focus);
            }

            /** \brief Get the state of search focusing */
            bool getFocusSearch() const
            {
                return getInformedSampling() && getPrunedMeasure() && getTreePruning() && getNewStateRejection();
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Set the number of attempts to make while performing rejection or informed sampling */
            void setNumSamplingAttempts(unsigned int numAttempts)
            {
                numSampleAttempts_ = numAttempts;
            }

            /** \brief Get the number of attempts to make while performing rejection or informed sampling */
            unsigned int getNumSamplingAttempts() const
            {
                return numSampleAttempts_;
            }

            unsigned int numIterations() const
            {
                return iterations_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }

        // protected:
            /** \brief Create the samplers */
            void allocSampler();

            /** \brief Generate a sample */
            bool sampleUniform(base::State *statePtr);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost> &costs, const base::OptimizationObjective &opt)
                  : costs_(costs), opt_(opt)
                {
                }
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i], costs_[j]);
                }
                const std::vector<base::Cost> &costs_;
                const base::OptimizationObjective &opt_;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const;

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
                Returns the number of motions pruned. Depends on the parameter set by
               setPruneStatesImprovementThreshold() */
            int pruneTree(const base::Cost &pruneTreeCost);

            /** \brief Computes the solution cost heuristically as the cost to come from start to the motion plus
                 the cost to go from the motion to the goal. If the parameter \e use_admissible_heuristic
                 (\e setAdmissibleCostToCome()) is true, a heuristic estimate of the cost to come is used;
                 otherwise, the current cost to come to the motion is used (which may overestimate the cost
                 through the motion). */
            base::Cost solutionHeuristic(const Motion *motion) const;

            /** \brief Add the children of a vertex to the given list. */
            void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);

            /** \brief Check whether the given motion passes the specified cost threshold, meaning it will be \e kept
             * during pruning */
            bool keepCondition(const Motion *motion, const base::Cost &threshold) const;

            /** \brief Calculate the k_RRG* and r_RRG* terms */
            void calculateRewiringLowerBounds();

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief An informed sampler */
            base::InformedSamplerPtr infSampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;




            /** \brief Option to use k-nearest search for rewiring */
            bool useKNearest_{true};

            /** \brief The rewiring factor, s, so that r_rrt = s \times r_rrt* > r_rrt* (or k_rrt = s \times k_rrt* >
             * k_rrt*) */
            double rewireFactor_{1.1};

            /** \brief A constant for k-nearest rewiring calculations */
            double k_rrt_{0u};

            /** \brief A constant for r-disc rewiring calculations */
            double r_rrt_{0.};

            /** \brief Option to delay and reduce collision checking within iterations */
            bool delayCC_{true};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The best goal motion. */
            Motion *bestGoalMotion_{nullptr};

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion *> goalMotions_;

            /** \brief The status of the tree pruning option. */
            bool useTreePruning_{false};

            /** \brief The tree is pruned when the change in solution cost is greater than this fraction. */
            double pruneThreshold_{.05};

            /** \brief Option to use the informed measure */
            bool usePrunedMeasure_{false};

            /** \brief Option to use informed sampling */
            bool useInformedSampling_{false};

            /** \brief The status of the sample rejection parameter. */
            bool useRejectionSampling_{false};

            /** \brief The status of the new-state rejection parameter. */
            bool useNewStateRejection_{false};

            /** \brief The admissibility of the new-state rejection heuristic. */
            bool useAdmissibleCostToCome_{true};

            /** \brief The number of attempts to make at informed sampling */
            unsigned int numSampleAttempts_{100u};

            /** \brief Option to create batches of samples and order them. */
            bool useOrderedSampling_{false};

            /** \brief The size of the batches. */
            unsigned int batchSize_{1u};

            /** \brief Stores the start states as Motions. */
            std::vector<Motion *> startMotions_;

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The cost at which the graph was last pruned */
            base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The measure of the problem when we pruned it (if this isn't in use, it will be set to
             * si_->getSpaceMeasure())*/
            double prunedMeasure_{0.};

            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0u};
            /** \brief Number of iterations the algorithm performed */
            unsigned int not_used_samples_{0u};

            ///////////////////////////////////////
            // Planner progress property functions
            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }
            std::string bestCostProperty() const
            {
                return std::to_string(bestCost().value());
            }
        };

    }  // namespace geometric
}  // namespace ompl

#endif