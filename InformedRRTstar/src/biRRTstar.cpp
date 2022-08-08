#include "../inc/biRRTstar.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include <time.h>
#include <string.h>
#include <chrono>
#include <thread>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
// #include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

ompl::geometric::biRRTstar::biRRTstar(const base::SpaceInformationPtr &si)
  : base::Planner(si, "biRRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &biRRTstar::setRange, &biRRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &biRRTstar::setGoalBias, &biRRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &biRRTstar::setRewireFactor, &biRRTstar::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &biRRTstar::setKNearest, &biRRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &biRRTstar::setDelayCC, &biRRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &biRRTstar::setTreePruning, &biRRTstar::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &biRRTstar::setPruneThreshold, &biRRTstar::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &biRRTstar::setPrunedMeasure, &biRRTstar::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &biRRTstar::setInformedSampling, &biRRTstar::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &biRRTstar::setSampleRejection, &biRRTstar::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &biRRTstar::setNewStateRejection,
                                &biRRTstar::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &biRRTstar::setAdmissibleCostToCome,
                                &biRRTstar::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &biRRTstar::setBatchSize, &biRRTstar::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &biRRTstar::setFocusSearch, &biRRTstar::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &biRRTstar::setNumSamplingAttempts,
                                        &biRRTstar::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::biRRTstar::~biRRTstar()
{
    freeMemory();
}

void ompl::geometric::biRRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    // distance setup
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();

    std::cout << "Setup..." << std::endl;
    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::geometric::biRRTstar::clear()
{
    std::cout << "biRRTstar clear..." <<std::endl;
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();

    bestGoalMotion_ = nullptr;
    bestConnectMotion_ = nullptr;
    connectMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    not_used_samples_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

ompl::base::PlannerStatus ompl::geometric::biRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    std::cout << "biRRTstar start to solve!" << std::endl;
    checkValidity();

    bool symCost = opt_->isSymmetric();//yes
    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)//pis_: PlannerInputStates (if many input states exist)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *smotion = new Motion(si_);
            si_->copyState(smotion->state, st);
            smotion->cost = opt_->identityCost();
            tStart_->add(smotion);// start 지점을 add
            startMotions_.push_back(smotion);
        }
        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    
    // add goal motion
    const base::State *gt = pis_.nextGoal(ptc);
    auto *gmotion = new Motion(si_);
    si_->copyState(gmotion->state, gt);
    gmotion->cost = opt_->identityCost(); // almost the cost is zero
    tGoal_->add(gmotion);
    
    // Checks that the start and goal poses are feasible
    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning goal tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
         allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *state_random = rmotion->state;
    base::State *xstate = si_->allocState();
    bool startTree = true;

    std::vector<Motion *> nbh;  // neighbors
    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;
    std::vector<int> valid;

    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    if (bestConnectMotion_)  // if there is a solution of cost
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_.value());

    if (useKNearest_)  // default, it is TRUE
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));
            // at sampling-based optimal motion planning for a non-holonlic dynamical system papers, IV-d

    // Our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);
    
    clock_t start_time = clock();
    while (ptc == false)
    {
        iterations_++;

        // Generate two trees
        TreeData &tree = startTree ? tStart_ : tGoal_;  // Ta
        startTree = !startTree;  // swap
        TreeData &otherTree = startTree ? tStart_ : tGoal_;  // Tb

        // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
        // loop and return to try again
        if (!sampleUniform(state_random)) {// state_random: random state
            continue;
        }

        // find the closest state in the current tree. using steering function.
        Motion *nmotion = tree->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, state_random))
            continue;


        base::State *state_new = state_random;
        // find state to add to the tree
        double d = si_->distance(nmotion->state, state_random);// using SteerFunction to compute a distance, from a tree to random node. //hc distance is used!        
        if (d > maxDistance_)// If the distance is larger than maxDistance_, it tries to build a (RS, HC, GC) path and cuts the path (interpolating step)
        {
            si_->getStateSpace()->interpolate(nmotion->state, state_random, maxDistance_ / d, xstate);
            state_new = xstate;// xstate: interpolate된 후, maxDistance_/d (=t) 값에 따라 획득된 motion값, state_new
        }   

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, state_new))// state_new is collision_free
        {
            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, state_new);
            motion->parent = nmotion;  // state_new's parent = nearest node
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);  // total_new_state_cost = nearest_node_cost + local_distance_cost
            
            // Find nearby neighbors of the new motion (state_new)
            getNeighbors(tree, motion, nbh);// steering function 사용.
            rewireTest += nbh.size();// rewireTest에 이웃 노드의 갯수를 삽입.
            ++statesGenerated;

            // cache for distance computations, sorting the costs
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())// 각 이웃 노드와, state_new (state_new)와의 cost 계산 필요.
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }
            
            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            /** \brief Choose Parent */
            // calculate all costs and local distances of neighbors
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);  // it uses statespace의 distance 사용 O, cost of {neighbor to state_new}
                costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);  // current cost of node + cost of {neighbor to state_new}
            }

            // sort the nodes
            //
            // we're using index-value pairs so that we can get at
            // original, unsorted indices // 이웃 노드와 cost가 존재하기에, 이웃 노드 중 valid한 노드 서치.
            for (std::size_t i = 0; i < nbh.size(); ++i)
                sortedCostIndices[i] = i;
            std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);  // To have sorting the index by cost

            for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                    i != sortedCostIndices.begin() + nbh.size(); ++i)
            {
                bool smaller_than_maxDistance;
                smaller_than_maxDistance = si_->distance(nbh[*i]->state, motion->state) < maxDistance_;

                if (nbh[*i] == nmotion ||
                    ((!useKNearest_ || smaller_than_maxDistance) &&
                        si_->checkMotion(nbh[*i]->state, motion->state)))
                {
                    motion->incCost = incCosts[*i];
                    motion->cost = costs[*i];
                    motion->parent = nbh[*i];
                    valid[*i] = 1;  // this neighbor node is valid (which means the new random node can be reached from the neighbor)
                    break;
                }
                else
                    valid[*i] = -1;
            }
            

            if (useNewStateRejection_)
            {// not used...
                if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))// if the new motion's cost could lower the best cost of the current path, the new motion is added.
                {
                    tree->add(motion);// tree에 add
                    motion->parent->children.push_back(motion);
                }
                else  // If the new motion does not improve the best cost it is ignored.
                {
                    si_->freeState(motion->state);
                    delete motion;
                    continue;
                }
            }
            else
            {// default
                // add motion to the tree
                tree->add(motion);
                motion->parent->children.push_back(motion);
            }

            /** \brief Rewiring step, this step takes super much time. */
            bool checkForSolution = false;
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {   // For a new state, it rewires neighbors,, i.e. the new state becomes a parent node of neighbors
                    base::Cost nbhIncCost;
                    if (symCost)  // it means, 'neighbor to state_new' or 'state_new to neighbor', all cost is same.
                        nbhIncCost = incCosts[i];  // incCost means the cost of {neighbor-to-state_new}
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);  // state_new to neighbor {new_cost}
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))  // If new_cost is better than the current, REWIRING!
                    {
                        bool motionValid;
                        if (valid[i] == 0)
                        {
                            motionValid =
                                (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) && 
                                si_->checkMotion(motion->state, nbh[i]->state);
                        }
                        else
                        {
                            motionValid = (valid[i] == 1);
                        }

                        if (motionValid)  // rewiring
                        {
                            // Remove this node from its parent list
                            removeFromParent(nbh[i]);
                            // Add this node to the new parent
                            nbh[i]->parent = motion; // state_new is a new parent
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);
                            checkForSolution = true;// rewiring occurs!, check the solution again!
                        }
                    }
                }
            }

            // pair<Motion *, Motion *> 
            ////////////////////////////////////////////////////////////////////////////////////////
            // bi-rrt*, main contribution
            // ConnectGraph in bi-RRTstar
            // find the closest state in the other tree. using steering function.
            Motion *connect_motion = otherTree->nearest(motion);  // motion is a state_new
            if (si_->equalStates(connect_motion->state, motion->state))
                continue;

            if (si_->checkMotion(connect_motion->state, motion->state)) {  // then solution is found!
                // create a motion
                auto *copied_motion = new Motion(si_);// This copied motion is as same as the motion above. 
                si_->copyState(copied_motion->state, motion->state);
                copied_motion->parent = connect_motion;//state_new가 x_new이므로, x_new의 parent는 nmotion
                copied_motion->incCost = opt_->motionCost(connect_motion->state, copied_motion->state);
                copied_motion->cost = opt_->combineCosts(connect_motion->cost, copied_motion->incCost);// total_new_state_cost
                copied_motion->inGoal = true;
                
                // Find a parent, step
                // Find nearby neighbors of the state_new, from otherTree
                getNeighbors(otherTree, copied_motion, nbh);  // X_near        
                if (costs.size() < nbh.size())
                {
                    costs.resize(nbh.size());
                    incCosts.resize(nbh.size());
                    sortedCostIndices.resize(nbh.size());
                }
                if (valid.size() < nbh.size())
                    valid.resize(nbh.size());
                std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

                // calculate all costs and local distances of neighbors
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, copied_motion->state);//모션 코스트 체크 시, statespace의 distance 사용 O,, cost of {neighbor to state_new}
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);// current cost of node + cost of {neighbor to state_new}
                }

                // sort the nodes
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);// To have sorting the index by cost

                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                        i != sortedCostIndices.begin() + nbh.size(); ++i)// cost 순으로 분류된 neighbor를 이용한 loop, sortedCostIndices는 이미 다른 vector들의 값이 들어와있을 수 있다.
                {
                    if (nbh[*i] == connect_motion ||// with neighboring...
                        (si_->checkMotion(nbh[*i]->state, copied_motion->state)))// 이웃 노드와 현재 모션까지의 checkMotion
                    {
                        copied_motion->incCost = incCosts[*i];
                        copied_motion->cost = costs[*i];
                        copied_motion->parent = nbh[*i];
                        valid[*i] = 1;// this neighbor node is valid (which means the new random node can be reached from the neighbor)
                        break;
                    }
                    else
                        valid[*i] = -1;
                }

                // No-rewiring
                auto * connected_motion = new std::pair<Motion *, Motion *>;
                connected_motion->first = motion;
                connected_motion->second = copied_motion;
                connectMotions_.push_back(connected_motion);
                checkForSolution = true;
            }
            ///////////////////////////////////////////////////////////////////////

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                if (!bestConnectMotion_ && !connectMotions_.empty())//current, bestConnectMotion_
                {   // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestConnectMotion_ = connectMotions_.front();
                    bestCost_ = opt_->combineCosts(bestConnectMotion_->first->cost, bestConnectMotion_->second->cost);// total path solution
                    std::cout << "The initial solution is found!" << std::endl;
                    std::cout << "Iteration: " << iterations_ << " | Time: " << (double)(clock() - start_time) / (double) CLOCKS_PER_SEC
                              << " | current solution cost: " << bestCost_.value()<< std::endl;
                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_.value(), iterations_, nn_->size());
                }
                else
                {
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &connectMotion : connectMotions_)// within all goal motions, checks the new one is better than the original?
                    {
                        /** \brief Best cost found so far by algorithm */
                        base::Cost tmpCost{std::numeric_limits<double>::quiet_NaN()};
                        tmpCost = opt_->infiniteCost();
                        tmpCost = opt_->combineCosts(connectMotion->first->cost, connectMotion->second->cost);

                        // Is this goal motion better than the (current) best?
                        if (opt_->isCostBetterThan(tmpCost, bestCost_))
                        {
                            bestConnectMotion_ = connectMotion;
                            bestCost_ = opt_->combineCosts(connectMotion->first->cost, connectMotion->second->cost);
                            // Check if it satisfies the optimization objective, if it does, break the for loop
                            if (opt_->isSatisfied(bestCost_))
                            {
                                break;
                            }
                        }
                    }
                }
                // no tree pruning
            }
            
        }
        else not_used_samples_++;// @MSK: 버려지는 샘플 찾기. (state_new가 valid하지 않은 경우!)

        // terminate if a sufficient solution is found
        if (bestConnectMotion_ && opt_->isSatisfied(bestCost_))
            break;
    }  // end while
    // std::cout << "while is done and not used samples / iterations: " << not_used_samples_ << " / " << iterations_ << std::endl;
    // std::cout << "using sample ratio [%]: " << 100.0 * (1.0 - (double)not_used_samples_ / (double)iterations_) << std::endl;

    // Add our solution (if it exists)
    std::pair<Motion *, Motion *>* newSolution = nullptr;
    if (bestConnectMotion_)
    {
        // We have an exact solution
        newSolution = bestConnectMotion_;
    }
    
    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        
        // construct the solution path
        std::vector<Motion *> final_path;
        std::vector<Motion *> mpath1;  // Assume mpath1 is connected to the start tree
        std::vector<Motion *> mpath2;
        Motion *iterMotion1 = newSolution->first;
        Motion *iterMotion2 = newSolution->second;
        bool is_first_goal_tree = false;
        while (iterMotion1 != nullptr)
        {
            mpath1.push_back(iterMotion1);
            if (iterMotion1->state->as<RSTATETYPE>()->getX() == gmotion->state->as<RSTATETYPE>()->getX() &&
                iterMotion1->state->as<RSTATETYPE>()->getY() == gmotion->state->as<RSTATETYPE>()->getY())
                is_first_goal_tree = true;
            iterMotion1 = iterMotion1->parent;  // parent를 따라 쭉~start까지
        }

        while (iterMotion2 != nullptr)
        {
            mpath2.push_back(iterMotion2);
            iterMotion2 = iterMotion2->parent;  // parent를 따라 쭉~start까지
        }

        if (!is_first_goal_tree) {
            std::reverse(mpath1.begin(), mpath1.end());// reverse the first path (connected to the start [assume])
            mpath1.insert(mpath1.end(), mpath2.begin() + 1, mpath2.end());// connect the two paths
            final_path = mpath1;
        }
        else {
            std::reverse(mpath2.begin(), mpath2.end());// second tree is connected to the start
            mpath2.insert(mpath2.end(), mpath1.begin() + 1, mpath1.end());// connect the two paths
            final_path = mpath2;
        }

        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        
        for (int i = 0; i < final_path.size(); i++) {
            path->append(final_path[i]->state);// path is generated from the start pose
        }

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestConnectMotion_)
            psol.setApproximate(approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, opt_->combineCosts(newSolution->first->cost, newSolution->second->cost), opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, connectMotions_.size(), bestCost_.value());
    std::cout << "THE FINAL COST IS " << bestCost_.value() << std::endl;
    // We've added a solution if newSolution == true, and it is an approximate solution if bestConnectMotion_ == false
    return {newSolution != nullptr, bestConnectMotion_ == nullptr};
}

void ompl::geometric::biRRTstar::getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(tree->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        tree->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        tree->nearestR(motion, r, nbh);  // is it using distance function?, yeah, it uses!
    }
}

void ompl::geometric::biRRTstar::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::biRRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::biRRTstar::freeMemory()
{
    std::vector<Motion *> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::biRRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
    }
    motions.clear();

    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }
}

int ompl::geometric::biRRTstar::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // // Remove it // TODO
                    // goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                    //                    goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::geometric::biRRTstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::biRRTstar::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::biRRTstar::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::geometric::biRRTstar::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::biRRTstar::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::biRRTstar::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::biRRTstar::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

// dd
// void ompl::geometric::biRRTstar::setOrderedSampling(bool orderSamples)
// {
//     // Make sure we're using some type of informed sampling
//     if (useInformedSampling_ == false && useRejectionSampling_ == false)
//     {
//         OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
//     }

//     // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
//     // one is already allocated.
//     if (orderSamples != useOrderedSampling_)
//     {
//         // Store the setting
//         useOrderedSampling_ = orderSamples;

//         // If we currently have a sampler, we need to make a new one
//         if (sampler_ || infSampler_)
//         {
//             // Reset the samplers
//             sampler_.reset();
//             infSampler_.reset();

//             // Create the sampler
//             allocSampler();
//         }
//     }
// }

void ompl::geometric::biRRTstar::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        OMPL_INFORM("%s: NOT USING INFORMED AND REJECTION SAMPLING.", getName().c_str());
        sampler_ = si_->allocStateSampler();
    }

    // sampler_->rng_.setLocalSeed(1);

    // Wrap into a sorted sampler
    // if (useOrderedSampling_ == true)
    // {
    //     infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    // }
    // No else
}

bool ompl::geometric::biRRTstar::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::biRRTstar::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}
