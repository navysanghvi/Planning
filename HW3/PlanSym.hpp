#ifndef PLANSYM_HPP
#define PLANSYM_HPP

#include <cmath>
#include <limits>
#include <random>
#include <list>
#include <iostream>

#include <math.h>

#include "planner_classes.cpp"
#include "GraphVertex.hpp"

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
    vector<vector<vector<string>>> GetActionSymSequencesUnique();
    GroundedCondition GetGroundedCondition(Condition* cond, vector<string> action_args, vector<string> value_args);
    GAction_t* GetGroundedAction(const Action* action, vector<string> arg_values);
    vector<GAction_t*> GetGroundedActions();
    bool IsValidAction(GState_t* current_state, GAction_t* action);
    bool Contains(GState_t* state, GState_t grounded_conditions);
    GState_t* AffectAction(GState_t* current_state, GAction_t* action);
    vector<pair<GState_t*,GAction_t*>> GetSuccessors(GState_t* current_state);
    int hashIndex(GState_t* state);
    bool IsGoal(GState_t* state);
    bool IsStart(GState_t* state);
    GraphVertex* GetToGoal();
    list<GAction_t> BacktrackPath(GraphVertex* goal);
    list<GAction_t> GetPlan();


protected:
    Env* m_env;
    vector<GAction_t*> m_grounded_actions;

};

#endif