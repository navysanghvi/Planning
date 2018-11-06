#ifndef PLANNERCLASSES_CPP
#define PLANNERCLASSES_CPP

#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

using namespace std;

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

class GroundedCondition
{
private:
    string predicate;
    vector<string> arg_values;
    bool truth;

public:
    GroundedCondition(string predicate, vector<string> arg_values)//, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = true;
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(string predicate, vector<string> arg_values, bool truth)
    {
        this->predicate = predicate;
        this->truth = truth;
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    vector<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth()
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size() || this->truth != rhs.truth)
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    vector<string> args;
    bool truth;

public:
    Condition(string pred, vector<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    vector<string> get_args() const
    {
        return this->args;
    }

    bool get_truth()
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size() || this->truth != rhs.truth)
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    vector<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, vector<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    vector<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;
    vector<string> symbols_v;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        this->symbols.insert(symbol);
        this->symbols_v.push_back(symbol);
    }
    void add_symbols(vector<string> symbols)
    {
        for (string l : symbols)
        {
            this->symbols.insert(l);
            this->symbols_v.push_back(l);
        }
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }
    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    vector<string> get_symbols_v() const
    {
        return this->symbols_v;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    vector<string> arg_values;

public:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects;

    GroundedAction(string name, vector<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(string name, vector<string> arg_values,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& preconditions,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& effects)
    {
        this->name = name;
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
        for (GroundedCondition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (GroundedCondition pc : effects)
        {
            this->effects.insert(pc);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    vector<string> get_arg_values() const
    {
        return this->arg_values;
    }
    
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

#endif