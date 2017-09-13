function [KC, KE, A, B, C] = static_reduction(K, ...
    internal_nodes, ext_loaded_nodes, contact_nodes, load_type)
% 
% [KC, KE, A, B, C] = static_reduction(K, internal_nodes, ext_loaded_nodes,
% contact_nodes, load_type) is a function that creates the reduced contact
% stiffness matrix (KC), the external load matric (KE), and the A, B, C
% matrices for frictional contact between a single elastic body and a 
% rigid plane obstacle.
% 
% INPUTS:   K: the global stiffness matrix of the single elastic body.
%           internal_nodes: the set of M-nodes that do NOT have any
% boundary conditions applied to them within ABAQUS, are NOT externally
% loaded, and are NOT along a contact interface.
%           ext_loaded_nodes: the set of M-nodes that are externally loaded
% either in force or displacement control (this cannot include some nodes 
% loaded in force control and others in displacement control, they must be
% all either force or displacement controlled).
%           contact_nodes: the set of M-nodes that are along a contact
% interface.
%           load_type: sets whether the external loads are applied in force
% or displacement control. Must be specified either as 'force' or as
% 'displ', respectively. Note that this setting affects both KC and KE.
% 
% OUTPUTS:  KC: the reduced contact stiffness matrix, organised with
% each column and row alternating between M-nodes of direction x and 
% direction y.
%           KE: the force matrix which is used to create the load vector fw
%           A: a repartitioned and reorganised part of the reduced contact
% stiffness matrix that connects contact forces of direction x with contact
% displacements of direction x
%           B: a repartitioned and reorganised part of the reduced contact
% stiffness matrix that connects contact forces of direction y with contact
% displacements of direction x
%           C: a repartitioned and reorganised part of the reduced contact
% stiffness matrix that connects contact forces of direction y with contact
% displacements of direction y
% 
% EXAMPLE: Peform static reduction on a global stiffness matrix (K) with
% externally loads applied in force control.
% [KC, KE, A, B, C] = static_reduction(K, ...
%     internal_nodes, ext_loaded_nodes, contact_nodes, 'force')

%================== Static Reduction =================%
% create the submatrices of the global stiffness matrix (K)
KII = K(internal_nodes, internal_nodes);
KIE = K(internal_nodes, ext_loaded_nodes); 
KEI = KIE';
KIC = K(internal_nodes, contact_nodes); 
KCI = KIC';
KEE = K(ext_loaded_nodes, ext_loaded_nodes);
KEC = K(ext_loaded_nodes, contact_nodes);
KCE = KEC';
KCC = K(contact_nodes, contact_nodes);
% create the L matrix and submatrices using the submatrices of K
L = [KEE,KEC;KCE,KCC]-[KEI;KCI]*(KII\[KIE,KIC]);
LEE = L(1:length(KEE(:,1)), 1:length(KEE(:,1)));
LEC = L(1:length(KEE(:,1)), length(KEE(:,1))+1:length(L(:,1))); 
LCE = LEC';
LCC = L(length(KEE(:,1))+1:length(L(:,1)), ...
    length(KEE(:,1))+1:length(L(:,1)));
%============== Make KC and KE Matrices ==============%
if strcmp(load_type,'force') == 1
    KC = full(LCC-LCE*(LEE\LEC));
    KE = full(LCE/LEE);
elseif strcmp(load_type,'displ') == 1
    KC = full(LCC);
    KE = full(LCE);
else
    error('Error: incorrect load_type selected. Must be ''force'' or ''displ''.')
end
%=============== Make A, B, C Matrices ===============%
degree_of_freedom_one = zeros(1,length(KC)/2);
degree_of_freedom_two = zeros(1,length(KC)/2);
for j = 1:length(KC)/2
    degree_of_freedom_one(1,j) = 2*(j-1)+1;
    degree_of_freedom_two(1,j) = 2*(j-1)+2;
end
A = KC(degree_of_freedom_one, degree_of_freedom_one);
B = KC(degree_of_freedom_two, degree_of_freedom_one);
C = KC(degree_of_freedom_two, degree_of_freedom_two);