function [KC, A, B, C] = merge_matrices(body_one_KC, body_two_KC)
% 
% [KC, A, B, C] = merge_matrices(body_one_KC, body_two_KC) is a function
% that merges the reduced contact stiffness matrices from two individual
% elastic bodies in contact with rigid plane obstacles, and creates the
% reduced contact matrix corresponding to the frictional contact between
% these two elastic bodies.
% 
% INPUTS:   body_one_KC: the reduced contact for the first body. This body
% CAN have rigid body degrees of freedom.
%           body_one_KC: the reduced contact for the first body. This body
% can NOT have rigid body degrees of freedom.
% 
% OUTPUTS:  KC: the reduced contact stiffness matrix for the case of
% frictional contact betwee the two elastic bodies, organised with each 
% column and row alternating between M-nodes of direction x and y.
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
% EXAMPLE: combine the reduced contact matrices from two separate elastic
% bodes.
%   [KC, A, B, C] = merge_matrices(body_one_KC, body_two_KC);
% 
% NOTES: The KC matrices of body 1 and body 2 must have the same number
% of contact nodes, and must be organised so that adjacent contact M-nodes
% have exactly the same position in the both matrices.

%================= Merge KC Matrices =================%
KC = (body_one_KC/body_two_KC + eye(size(body_two_KC)))\body_one_KC;
%============== Create A, B, C Matrices ==============%
degree_of_freedom_one = zeros(1,length(KC)/2);
degree_of_freedom_two = zeros(1,length(KC)/2);
for j = 1:length(KC)/2
    degree_of_freedom_one(1,j) = 2*(j-1)+1;
    degree_of_freedom_two(1,j) = 2*(j-1)+2;
end
A = KC(degree_of_freedom_one, degree_of_freedom_one);
B = KC(degree_of_freedom_two, degree_of_freedom_one);
C = KC(degree_of_freedom_two, degree_of_freedom_two);
