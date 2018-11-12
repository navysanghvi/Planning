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

using namespace std;

class PlanSym
{
public:
    PlanSym(Env* environment, int heurType);
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

    bool Contains(GState_t* state, GState_t grounded_conditions);
    int GoalTrueCondsUnsatisfied(GState_t* state);
    int NumTrueCondsUnsatisfied(GState_t* state, GState_t grounded_conditions);
    bool IsValidAction(GState_t* current_state, GAction_t* action);
    bool IsGoal(GState_t* state);
    bool IsStart(GState_t* state);

    GState_t* AffectAction(GState_t* current_state, GAction_t* action, bool emptyDeleteList = false);
    vector<pair<GState_t*,GAction_t*>> GetSuccessors(GState_t* current_state, bool emptyDeleteList = false);
    

    void GetHeuristicValue(GraphVertex* start, int heurType);
    int hashIndex(GState_t* state);
    GraphVertex* GetToGoal(int heurType = 0, GraphVertex* start = NULL, bool emptyDeleteList = false);
    
    list<GAction_t> BacktrackPath(GraphVertex* goal);
    list<GAction_t> GetPlan();


protected:
    Env* m_env;
    vector<GAction_t*> m_grounded_actions;
    int m_numStatesExpanded;
    int m_heurType;

};

#endif