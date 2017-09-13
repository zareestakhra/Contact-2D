function [fw_unsorted] = create_two_body_fw( ...
    body_one_KC, body_two_KC, body_one_fw, body_two_fw)
% 
% [load_vector] = create_two_body_fw(body_one_KC, body_two_KC, 
% body_one_fw, body_two_fw) is a function that can manipulate a load vector
% fw for the case of contact between two elastic bodies.
% 
% INPUTS:   body_one_KC: the reduced contact stiffness matrix of the first
% elastic body.
%           body_two_KC: the reduced contact stiffness matrix of the second
% elastic body.
%           body_one_fw: the unsorted loading vector (organised to match 
% KC)from the first elastic body. If no load is to be applied to the first 
% body then input 'none'.
%           body_one_fw: the unsorted loading vector (organised to match 
% KC)from the second elastic body.  If no load is to be applied to the
% second body then input 'none'.
% 
% EXAMPLE: Apply a load to body one but not body two of the two elastic 
% bodies in contact.
%     [load_vector] = create_two_body_fw(body_one_KC, body_two_KC, ...
%         body_one_fw, 'none');
% 
% NOTES: Unless loads are to be applied simultaneously to both bodies, then
% either body_one_fw or body_two_fw should be specified as 'none'. This is
% likely to be the most common loading scenario.

%========== Create fw For Two Elastic Bodies =========%
if strcmp(body_one_fw,'none') == 1;
    body_one_fw = zeros(length(body_one_KC(:,1)),1);
elseif strcmp(body_two_fw,'none') == 1;
    body_two_fw = zeros(length(body_two_KC(:,1)),1);
else
end
fw_unsorted = (body_one_KC/body_two_KC + eye(size(body_two_KC)))\ ...
     (body_one_fw + body_one_KC*(body_two_KC\body_two_fw));