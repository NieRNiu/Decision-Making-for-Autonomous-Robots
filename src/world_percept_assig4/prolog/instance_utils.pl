%%
%% Copyright (C) 2010 by Karinne Ramirez-Amaro
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
/* ***************************************
	Author:	Karinne Ramirez-Amaro
	E-mail:	karinne@chalmers.se
	
 This library contains predicates used for the
 inference method using prolog queries.


 NOTE: The following symbols are used to describe the parameter
of the predicates
 + The argument is instantiated at entry.
 - The argument is not instantiated at entry.
 ? The argument is unbound or instantiated at entry.

*/

:- module(instance_utils,
    [
	create_instance_from_class/3,
	getClassPath/2,
	get_class/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(ssy236Ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).

%%%%%%%%%%%%%% Custom computables %%%%%%%%%%%%%%%%%%%%%%

% This function will create an instance of a desired class
% create_instance_from_class(+Class, +Instance_ID, ?Instance)
% The created instance will have the predicate/property rdf:type
% to correctly inheritate the properties of its super-classes
%
% @param Class		represents the name of the class where the instance will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Instance_ID	is the new ID that the instance will have
% @param Instance	asserted new instance

% TODO: This prolog query should check if the class we want to insert exist in the ontology. IF the class does not exits it should create a parent class, then the new instance.

create_instance_from_class(Class, Instance_ID, Instance) :-
	% Check ID and class path
	getClassPath(Class,Class_path),
	
	% Create the path of the new instance
	atom_concat(Class_path,  '_', Class2),
	atomic_concat(Class2, Instance_ID, Instance),
	% write(Instance),nl,
	% assert/create the new instance
	rdf_assert(Instance, rdf:type, Class_path).

% This function will return the path of the class/instance given
% getClassPath(+Class, ?Class_path)
%
% @param Class		represents the name of the class where the instance will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Class_path	correct class full path

getClassPath(Class, Class_path):-
	((concat_atom(List, '#', Class),length(List,Length),Length>1) ->
	( % Class has already a URI
	   Class_path=Class );
	  % When the class does not have URL, will get the knowrob path
        ( TempClass='http://www.chalmers.se/ontologies/ssy236Ontology.owl#',
	atom_concat(TempClass, Class, Class_path)
	% write(Class_path), nl
 	)).

% This function will return the path of the obtained class
% get_class(+Class)
%
% @param Class		represents the name of the class that we ask to create (if it does not exsist yet).
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"

% TODO: Properly define this predicate to check if  class already exists in the ontology ssy236Ontology. If the class does not exist, we need to assert a new one, otherwise nothing will happen.
get_class(Class) :-
    % Define the ontology prefix
    OntologyPrefix = 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#',
    
    % Construct the full class URI
    atom_concat(OntologyPrefix, Class, FullClassPath),
    
    % Check if the class exists in the ontology
    (   rdf_has(FullClassPath, rdf:type, owl:'Class')
    ->  % If the class exists, do nothing
		(fail)
    ;   % If the class does not exist, create it
        (   rdf_assert(FullClassPath, rdf:type, owl:'Class'),
            write('New class created: '), write(FullClassPath), nl
        )
    ).



