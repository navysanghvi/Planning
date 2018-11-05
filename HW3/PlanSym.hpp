#ifndef PLANSYM_HPP
#define PLANSYM_HPP

#include <cmath>
#include <limits>
#include <random>
#include <list>
#include <iostream>

#include <math.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>

#include "planner_classes.cpp"

#define D_INF numeric_limits<double>::infinity()

using namespace std;

class PlanSym
{
public:
    PlanSym(Env* environment);
    PlanSym();
    ~PlanSym();

    typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> GState_t;
    typedef unordered_set<Action, ActionHasher, ActionComparator> Actions_t;
    typedef GroundedAction GAction_t;
    typedef unordered_set<string> Sym_t;

    vector<vector<vector<string>>> GetActionSymSequences();
    bool IsValidAction(GState_t* current_state, GAction_t* action);
    GState_t AffectAction(GState_t* current_state, GAction_t* action);
    GroundedCondition GetGroundedCondition(Condition* cond, vector<string> action_args, vector<string> value_args);
    GAction_t GetGroundedAction(Action* action, vector<string> arg_values);
    vector<GState_t> GetSuccessors(GState_t* current_state);


protected:
    Env* m_env;
    vector<vector<vector<string>>> m_actionSymSeqs;

};

#endif