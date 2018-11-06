#include <queue>
#include <vector>

#include <time.h>

#include "PlanSym.hpp"
#include "Compare.cpp"
#include "GraphVertex.hpp"

PlanSym::PlanSym(Env* environment)
{
	m_env = environment;
	m_grounded_actions = GetGroundedActions();
}

vector<vector<vector<string>>> PlanSym::GetActionSymSequences()
{
	Actions_t envActions = m_env->get_actions();
	vector<string> envSyms = m_env->get_symbols_v();
	int sym_sz, envSym_sz = envSyms.size();
	vector<vector<vector<string>>> actionSymSeqs;

	for(auto actIt = envActions.begin(); actIt != envActions.end(); actIt++)
	{
		int action_args_sz = ((*actIt).get_args()).size();
		vector<vector<string>> symSeqs(pow(envSym_sz, action_args_sz));

		sym_sz = symSeqs.size();
		for(int i = 0; i < action_args_sz; i++)
		{
			sym_sz /= envSym_sz;
			for(int j = 0; j < symSeqs.size(); j++)
				symSeqs[j].push_back(envSyms[(int(j/sym_sz))%envSym_sz]);
		}
		actionSymSeqs.push_back(symSeqs);
	}

	return actionSymSeqs;
}


vector<vector<vector<string>>> PlanSym::GetActionSymSequencesUnique()
{
	vector<vector<vector<string>>> actionSymSeqs = GetActionSymSequences();
	vector<vector<vector<string>>> actionSymSeqsUnique;

	for(int i = 0; i < actionSymSeqs.size(); i++)
	{
		vector<vector<string>> symSeqs;
		for(int j = 0; j < actionSymSeqs[i].size(); j++)
		{
			unordered_map<string,bool> exists; int k = 0;
			for(; k < actionSymSeqs[i][j].size(); k++)
			{
				if(exists[actionSymSeqs[i][j][k]]){break;}
				exists[actionSymSeqs[i][j][k]] = 1;
			}
			if(k == actionSymSeqs[i][j].size())
				symSeqs.push_back(actionSymSeqs[i][j]); 
		}
		actionSymSeqsUnique.push_back(symSeqs);
	}
	return actionSymSeqsUnique;

}


GroundedCondition PlanSym::GetGroundedCondition(Condition* cond, vector<string> action_args, vector<string> value_args)
{
	vector<string> ground_args; bool insert;
	for(string c_arg: cond->get_args())
	{
		insert = 0;
		for(int i = 0; i < action_args.size(); i++)
		{
			if(c_arg == action_args[i])
			{
				ground_args.push_back(value_args[i]);
				insert = 1;
			}
		}
		if(!insert){ ground_args.push_back(c_arg); }
	}
	return GroundedCondition(cond->get_predicate(), ground_args, cond->get_truth());
}

PlanSym::GAction_t* PlanSym::GetGroundedAction(const Action* action, vector<string> value_args)
{
	GAction_t* grounded_action = new GAction_t(action->get_name(), value_args);
	GState_t preconditions, effects;
	vector<string> action_args = action->get_args();
	
	for(Condition pc: action->get_preconditions())
		preconditions.insert(GetGroundedCondition(&pc, action_args, value_args));
	grounded_action->preconditions = preconditions;

	for(Condition e: action->get_effects())
		effects.insert(GetGroundedCondition(&e, action_args, value_args));
	grounded_action->effects = effects;

	return grounded_action;
}

vector<PlanSym::GAction_t*> PlanSym::GetGroundedActions()
{
	vector<GAction_t*> grounded_actions;
	Actions_t envActions = m_env->get_actions();
	auto actIt = envActions.begin();
	//vector<vector<vector<string>>> actionSymSeqs = GetActionSymSequences();
	vector<vector<vector<string>>> actionSymSeqs = GetActionSymSequencesUnique();

	for(int i = 0; i < actionSymSeqs.size(); i++)
	{
		const Action* action = &(*actIt);
		for(int j = 0; j < actionSymSeqs[i].size(); j++)
			grounded_actions.push_back(GetGroundedAction(action, actionSymSeqs[i][j]));
		actIt++;
	}
	return grounded_actions;
}

bool PlanSym::IsValidAction(GState_t* current_state, GAction_t* action)
{
	return Contains(current_state, action->get_preconditions());
}

bool PlanSym::Contains(GState_t* state, GState_t grounded_conditions)
{
	bool truth; int count; 
	for(GroundedCondition gc: grounded_conditions)
	{
		truth = gc.get_truth(); count = state->count(gc);
		if((truth && !count) ||(!truth && count)) { return false; }
	}
	return true;
}

PlanSym::GState_t* PlanSym::AffectAction(GState_t* current_state, GAction_t* action)
{
	GState_t* next_state = new GState_t(*current_state);
	for(GroundedCondition e: action->get_effects())
	{
		if(e.get_truth()) { next_state->insert(e); }
		else { next_state->erase(GroundedCondition(e.get_predicate(), e.get_arg_values(), true)); }
	}
	return next_state;
}

vector<pair<PlanSym::GState_t*,PlanSym::GAction_t*>> PlanSym::GetSuccessors(GState_t* current_state)
{
	vector<pair<GState_t*,GAction_t*>> successor_states;
	for(GAction_t* action: m_grounded_actions)
	{
		if(IsValidAction(current_state, action))
			successor_states.push_back(pair<GState_t*, GAction_t*>(AffectAction(current_state, action),action));
	}

	return successor_states;
}

int PlanSym::hashIndex(GState_t* state)
{
	int ind = 0;
	for(auto gcondIt = state->begin(); gcondIt != state->end(); gcondIt++)
		ind += hash<string>{}((*gcondIt).toString());
	return ind;
}

bool PlanSym::IsGoal(GState_t* state)
{
	return Contains(state, m_env->get_goal_conditions());
}

bool PlanSym::IsStart(GState_t* state)
{
	return Contains(state, m_env->get_initial_conditions());
}

GraphVertex* PlanSym::GetToGoal()
{
	priority_queue<GraphVertex*, vector<GraphVertex*>, Compare> open;
	unordered_map<GraphVertex*, bool> closed;
	unordered_map<int, GraphVertex*> graph;
	vector<pair<GState_t*,GAction_t*>> successors; 
	int gInd; double pCost;

	GraphVertex* curr = new GraphVertex(new GState_t(m_env->get_initial_conditions()), 0);
	open.push(curr);
	graph[hashIndex(curr->m_state)] = curr;

	m_numStatesExpanded = 0;

	while(!open.empty())
	{
		while(closed[open.top()]) { open.pop(); }
		curr = open.top(); closed[curr] = 1; open.pop();
		pCost = curr->m_pCost;

		m_numStatesExpanded++;

		if(IsGoal(curr->m_state)){ return curr; }
		successors = GetSuccessors(curr->m_state);
		for(int i = 0; i < successors.size(); i++)
		{
			gInd = hashIndex(successors[i].first);
			if(graph[gInd] == NULL)
			{
				graph[gInd] = new GraphVertex(successors[i].first, pCost+1, curr, successors[i].second);
				open.push(graph[gInd]);
			}
			else
			{
				graph[gInd]->m_parents.push_back(curr);
				graph[gInd]->m_parActions.push_back(successors[i].second);
				if(graph[gInd]->m_pCost > pCost+1){ graph[gInd]->m_pCost = pCost+1; open.push(graph[gInd]); }
			}
		}
	}
	return NULL;
}

list<PlanSym::GAction_t> PlanSym::BacktrackPath(GraphVertex* goal)
{
	list<GAction_t> planActions;
	GraphVertex* curr = goal;
	while(!IsStart(curr->m_state))
	{
		for(int i = 0; i < curr->m_parents.size(); i++)
		{
			if((curr->m_parents[i])->m_pCost + 1 <= curr->m_pCost)
			{
				planActions.push_front(*(curr->m_parActions[i]));
				curr = curr->m_parents[i];
				break;
			}
		}
	}
	return planActions;
}

list<PlanSym::GAction_t> PlanSym::GetPlan()
{
	clock_t begin_time = clock();
	GraphVertex* goal = GetToGoal();
	if(goal == NULL){ cout << "Didn't get to goal\n"; return list<GAction_t>(); }

	cout << "Cost to goal = " << goal->m_pCost << '\n';
	cout << "Number of States Expanded = " << m_numStatesExpanded << '\n';
	cout << "Planning Time = " << double(clock() - begin_time)/CLOCKS_PER_SEC << " s\n";
	return BacktrackPath(goal);
}