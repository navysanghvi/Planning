#include "PlanSym.hpp"

PlanSym::PlanSym(Env* environment)
{
	m_env = environment;
	m_actionSymSeqs = GetActionSymSequences();
}

bool PlanSym::IsValidAction(GState_t* current_state, GAction_t* action)
{
	bool truth; int count; 
	for(GroundedCondition pc: action->get_preconditions())
	{
		truth = pc.get_truth(); count = current_state->count(pc);
		if((truth && !count) ||(!truth && count)) { return false; }
	}
	return true;
}

PlanSym::GState_t PlanSym::AffectAction(GState_t* current_state, GAction_t* action)
{
	GState_t next_state = *current_state;
	for(GroundedCondition e: action->get_effects())
	{
		if(e.get_truth()) { next_state.insert(e); }
		else { next_state.erase(e); }
	}
	return next_state;
}

PlanSym::GAction_t PlanSym::GetGroundedAction(Action* action, vector<string> value_args)
{
	GAction_t grounded_action(action->get_name(), value_args);
	GState_t preconditions, effects;
	vector<string> action_args = action->get_args();
	
	for(Condition pc: action->get_preconditions())
		preconditions.insert(GetGroundedCondition(&pc, action_args, value_args));
	grounded_action.preconditions = preconditions;

	for(Condition e: action->get_effects())
		effects.insert(GetGroundedCondition(&e, action_args, value_args));
	grounded_action.effects = effects;

	return grounded_action;
}

GroundedCondition PlanSym::GetGroundedCondition(Condition* cond, vector<string> action_args, vector<string> value_args)
{
	vector<string> gound_args;
	for(string c_arg: cond->get_args())
		for(int i = 0; i < action_args.size(); i++)
			if(c_arg == action_args[i])
				gound_args.push_back(value_args[i]);
	return GroundedCondition(cond->get_predicate(), gound_args, cond->get_truth());
}

vector<PlanSym::GState_t> PlanSym::GetSuccessors(GState_t* current_state)
{
	vector<GState_t> successor_states;
	vector<GAction_t*> actions;// = GetActions();
	for(GAction_t* action: actions)
		if(IsValidAction(current_state, action))
			successor_states.push_back(AffectAction(current_state, action));

	return successor_states;
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
