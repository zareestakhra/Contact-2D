function [matlab_stiffness_matrix] = import_stiffness_matrix(mtx_file)
% 
% matlab_stiffness_matrix = import_stiffness_matrix(mtx_file) is a
% function that imports the global stiffness matrix from a finite element
% model that has been created in ABAQUS/CAE. The global stiffness matrix
% must already have been extracted from ABAQUS and be contained in a .mtx
% file in MATLAB's working directory for this function to be used.
% 
% INPUT:    mtx_file: the path to the .mtx file, e.g. 'matrix_file.mtx'
% 
% OUTPUT:   matlab_stiffness_matrix: a sparse matrix containing the data in
% the .mtx file. This sparse matrix is numbered using a different numbering
% convention than is used in the ABAQUS model.
% 
% EXAMPLE:  Import the global stiffness matrix from a .mtx file named 
% 'matrix_file.mtx'.
%   matlab_stiffness_matrix = import_stiffness_matrix('matrix_file.mtx');
% 
% NOTES: The stiffnesss matrix contained in the .mtx file is a [N x 5] 
% matrix where N is the number of non-zero stiffness values in the global
% stiffness matrix. Each row in this file lists the stiffness between two
% degrees of freedom in the finite element model. The stiffness value is
% contained in the 5th column of the .mtx file. Columns 1 and 2 of the 
% .mtx file contain the numbers of two nodes that the stiffness value
% connects. Columns 3 and 4 specify which degree of freedom from the nodes
% specified columns 1 and 2 are connected by the stiffness value in column
% 5. The sparse matrix that is output by this file is organised such that
% the value of each element is a stiffness value and the row and column
% numbers of the particular element correspond to the degrees of freedom
% that it connects. The rows are numbered such that odd numbers correspond
% to degrees of freedom in the X-direction, and even numbers correpond to
% degrees of freedom in the Y-direction.

%============== Import Stiffness Matrix ==============%
abaqus_stiffness_matrix = dlmread(mtx_file); 
% merge node number info from column 1 and DOF info from column 2 and 
% store in the 1st column of a new matrix
matlab_nodes(:,1) = 2*(abaqus_stiffness_matrix(:,1)-1)+ ...
    abaqus_stiffness_matrix(:,2); 
% merge node number info from column 3 and DOF info from column 4 and 
% store in the 2nd column of a new matrix
matlab_nodes(:,2) = 2*(abaqus_stiffness_matrix(:,3)-1)+ ...
    abaqus_stiffness_matrix(:,4); 
% extract the stiffness values from the .mtx file, and store in a double 
% length vector
stiffness_values = [abaqus_stiffness_matrix(:,5); ...
    abaqus_stiffness_matrix(:,5)];

% create a matrix of the new matlab node numbers, and a vector of indices 
% of their position in the abaqus stiffness matrix
[matlab_matrix_indices, abaqus_stiffness_value_index] = unique( ...
    [matlab_nodes; matlab_nodes(:,2) matlab_nodes(:,1)], 'rows'); 
% compile the stiffness matrix using the new node numbering convention
matlab_stiffness_matrix = accumarray( matlab_matrix_indices, ...
    stiffness_values(abaqus_stiffness_value_index), [], @max, [], true);