
:- module(decision_utils,
    [
        should_pick_up/1,
        prioritize_target/2,
        plan_action/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

%% Define namespaces
:- rdf_db:rdf_register_ns(ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).

%%%%%%%%%%%%%%% Decision-Making Predicates %%%%%%%%%%%%%%%%%%

%% should_pick_up(+Object)
%% Decides if the robot should pick up an object based on its type or task requirements.
%% @param Object    The object to evaluate.
should_pick_up(Object) :-
    rdf(Object, rdf:type, ontology:graspable_object),
    rdf(Object, ontology:importance, Importance),
    Importance > 0.5, % Threshold for importance
    write('Decision: Pick up '), write(Object), nl.

%% prioritize_target(+TargetList, -TopPriority)
%% Determines the highest-priority target from a list of possible targets.
%% @param TargetList   A list of targets to prioritize.
%% @param TopPriority  The target with the highest priority.
prioritize_target(TargetList, TopPriority) :-
    findall([Target, Priority],
            (member(Target, TargetList), rdf(Target, ontology:priority, Priority)),
            PrioritizedList),
    sort(2, @>=, PrioritizedList, SortedList), % Sort by priority (descending)
    nth0(0, SortedList, [TopPriority, _]),     % Select the first item as the top priority
    write('Decision: Prioritize '), write(TopPriority), nl.

%% plan_action(+CurrentState, -Action)
%% Determines the next action to take based on the current state.
%% @param CurrentState   The current state of the environment or robot.
%% @param Action         The planned action to execute.
plan_action(CurrentState, Action) :-
    (   CurrentState = 'object_detected' -> Action = 'grasp_object';
        CurrentState = 'object_missing'  -> Action = 'search_for_object';
        CurrentState = 'task_complete'   -> Action = 'return_to_base';
        Action = 'idle' % Default action
    ),
    write('Decision: Execute action '), write(Action), nl.
