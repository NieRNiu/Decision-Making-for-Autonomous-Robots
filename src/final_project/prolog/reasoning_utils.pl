
:- module(reasoning_utils,
    [
        near_object/3,
        is_on_surface/2,
        navigate_to_likely_location/2,
        check_object_state/2,
        add_object_location/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

%% Define namespaces
:- rdf_db:rdf_register_ns(ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).

%%%%%%%%%%%%%%% Reasoning Predicates %%%%%%%%%%%%%%%%%%

%% near_object(+Object1, +Object2, ?Relation)
%% This predicate infers the spatial relation between two objects.
%% @param Object1    First object.
%% @param Object2    Second object.
%% @param Relation   Spatial relation (e.g., 'close_to', 'far_from').
near_object(Object1, Object2, Relation) :-
    % Dummy condition to infer proximity
    rdf(Object1, ontology:distance_to, Distance),
    (Distance < 1.0 -> Relation = close_to ; Relation = far_from).

%% is_on_surface(+Object, +Surface)
%% Checks if an object is likely located on a given surface based on known knowledge.
%% @param Object    The object to check.
%% @param Surface   The surface to check against.
is_on_surface(Object, Surface) :-
    rdf(Object, ontology:located_on, Surface).

%% navigate_to_likely_location(+Object, ?Location)
%% Predicts a likely location for an object based on learned probabilities or pre-defined knowledge.
%% @param Object    The object to locate.
%% @param Location  Likely location.
navigate_to_likely_location(Object, Location) :-
    % Example knowledge assertion for a bottle likely being on a table
    (rdf(Object, rdf:type, ontology:bottle) ->
        Location = 'big_table';
        Location = 'unknown_location').

%% check_object_state(+Object, ?State)
%% Determines the state of an object (e.g., open, closed).
%% @param Object    The object to check.
%% @param State     The state of the object.
check_object_state(Object, State) :-
    rdf(Object, ontology:has_state, State).

%%%%%%%%%%%%%%% Dynamic Knowledge Update %%%%%%%%%%%%%%%%%%

%% add_object_location(+Object, +Location)
%% Dynamically asserts the location of an object in the ontology.
%% @param Object    The object to add.
%% @param Location  The location of the object.
add_object_location(Object, Location) :-
    rdf_assert(Object, ontology:located_on, Location).
