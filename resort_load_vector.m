function [load_vector] = resort_load_vector(fw_load_vector)
%================= Re-sort Node Set ==================%
load_vector = zeros(length(fw_load_vector), 1);
% create a vector of even rows and of odd rows
for j = 1:length(load_vector)/2
load_vector(j, 1) = fw_load_vector(2*(j-1)+1, 1);
load_vector(length(load_vector)/2+j, 1) = fw_load_vector(2*(j-1)+2, 1);
end