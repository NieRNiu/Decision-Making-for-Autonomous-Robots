:- module(my_predicates,
    [
    assert_OnTop/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(ssy236Ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).


% getClassPath(Inst1, I1),
% getClassPath(Inst2, I2),
% rdf_assert(I1, ssy236Ontology:'onTop',n I2).

assert_OnTop(Inst1, Inst2) :-
    % Assuming getClassPath/2 is a valid predicate that retrieves I1 and I2
    getClassPath(Inst1, I1),
    getClassPath(Inst2, I2),
    rdf_assert(I1, ssy236Ontology:'onTop', I2).