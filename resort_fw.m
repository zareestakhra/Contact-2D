function [fw_organised] = resort_fw(fw_unsorted)
% 
% [fw_organised] = resort_fw(fw_unsorted) is a function that reorganises a
% load vector fw that has been created either by the function
% create_fw or create_two_body_fw to be compatible with the A, B, C
% matrices.
% 
% INPUT:    fw_unsorted: a load vector that is organised to be compatible
% with the KC matrix.
% 
% OUTPUT:   fw_organised: a load vector that is organised to be compatible
% with the A, B, C matrices. 
% 
% EXAMPLE: reorganise a load vector.
%   [fw_organised] = resort_fw(fw_unsorted);

%==================== Re-sort fw =====================%
fw_organised = zeros(length(fw_unsorted), 1);
% create a vector of even rows and of odd rows
for j = 1:length(fw_organised)/2
    fw_organised(j, 1) = fw_unsorted(2*(j-1)+1, 1);
    fw_organised(length(fw_organised)/2+j, 1) = fw_unsorted(2*(j-1)+2, 1);
end
end
