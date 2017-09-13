function [fw_unsorted] = create_fw( KE, ...
    load_distribution_x, load_distribution_y)
%
% [fw_load_vector] = create_fw( KE, load_distribution_x, 
% load_distribution_y) is a function that creates the load vector fw,
% which gives the contact forces that result from an externally load. It
% is not important whether the load is applied in force or displacement
% control, this is accounted for in the KE matrix. The load distribution
% represents external displacements if KE was created with the option
% 'displ' or external forces if KE was created with the option 'force'.
% 
% INPUTS:    KE: the external load matrix that is multiplied by the load
% dirstibution to create the load vector fw.
%           load_distribution_x: describes the load distribution that
% is applied to M-nodes of degree of freedom 1. 
%           load_distribution_y: describes the load distribution that
% is applied to M-nodes of degree of freedom 2. 
% 
% The options for load_distribution_x and load_distribution_y are:
%   -zero applied load 'zero'
%   -constant +1 load 'constant'
%   -constant -1 load '-constant'
%   -linearly varying load with positive slope which integrates to 
% zero 'linear'
%   -linearly varying load with negative slope '-linear'
%   -a step function with the left half +1 and the right half -1 'pmstep'
%   -a step function with left the half -1 and the right half +1 'mpstep'
% 
% OUTPUTS:  fw_unsorted: the load vecor fw corresponding to the load
% distributions selected. Note that this load vector is organised to match
% the KC matrix and NOT the A, B, C matrices.
% 
% EXAMPLE: Create a load vector of uniform applied load in the negative y
% direction, and zero load in the x direction.
%   fw_unsorted = create_fw( KE, 'zero', '-constant');

%============= Create Load Distributions =============%
load_zero = zeros(length(KE(1,:))/2,1);
load_constant = ones(length(KE(1,:))/2,1);
load_linear = zeros(length(KE(1,:))/2,1); %preallocate
for i = 0:length(load_linear)-1
   load_linear(i+1,1) = -1 + 2*(i/(length(load_linear)-1));
end
%========= Select Load Distribution for DOF 1 ========%
if strcmp(load_distribution_x,'zero') == 1;
    load_distribution_x = load_zero;
elseif strcmp(load_distribution_x,'constant') == 1;
    load_distribution_x = load_constant;
elseif strcmp(load_distribution_x,'-constant') == 1;
    load_distribution_x = -load_constant;
elseif strcmp(load_distribution_x,'linear') == 1;
    load_distribution_x = load_linear;
elseif strcmp(load_distribution_x,'-linear') == 1;
    load_distribution_x = -load_linear;
elseif strcmp(load_distribution_x,'pmstep') == 1;
    load_pmstep = [ones(length(KE(1,:))/4,1); ...
        -ones(length(KE(1,:))/4,1)];
    load_distribution_x = load_pmstep;
elseif strcmp(load_distribution_x,'mpstep') == 1;
    load_mpstep = [-ones(length(KE(1,:))/4,1); ...
        ones(length(KE(1,:))/4,1)];
    load_distribution_x = load_mpstep;
else
    error('Error: incorrect load distribution selected for ''y'' direction')
end
%========= Select Load Distribution for DOF 2 ========%
if strcmp(load_distribution_y,'zero') == 1;
    load_distribution_y = load_zero;
elseif strcmp(load_distribution_y,'constant') == 1;
    load_distribution_y = load_constant;
elseif strcmp(load_distribution_y,'-constant') == 1;
    load_distribution_y = -load_constant;
elseif strcmp(load_distribution_y,'linear') == 1;
    load_distribution_y = load_linear;
elseif strcmp(load_distribution_y,'-linear') == 1;
    load_distribution_y = -load_linear;
elseif strcmp(load_distribution_y,'pmstep') == 1;
    load_pmstep = [ones(length(KE(1,:))/4,1); ...
        -ones(length(KE(1,:))/4,1)];
    load_distribution_y = load_pmstep;
elseif strcmp(load_distribution_y,'mpstep') == 1;
    load_mpstep = [-ones(length(KE(1,:))/4,1); ...
        ones(length(KE(1,:))/4,1)];
    load_distribution_y = load_mpstep;
else
    error('Error: incorrect load distribution selected for ''x'' direction')
end
%===================== Create fw =====================%
load_distribution = zeros(2*length(load_distribution_x),1);
for j = 1:(length(KE(1,:))/2)
    load_distribution(2*(j-1)+1, 1) = load_distribution_x(j,1);
    load_distribution(2*(j-1)+2, 1) = load_distribution_y(j,1);
end
fw_unsorted = KE*load_distribution;