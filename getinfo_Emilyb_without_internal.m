function [Anodes_contact_bodyB, KCB, KEB]=getinfo_Emilyb_without_internal()

clear all;

load('EmilyData.mat');

numNode_Block=length(K)/(6*2);

%% DOF1
NodeNumber1=[1:numNode_Block];
for j = 1:length(NodeNumber1)
        DOF1(1, 2*(j-1)+1) = 6.*(NodeNumber1(1,j)-1)+1;
        DOF1(1, 2*(j-1)+2) = 6.*(NodeNumber1(1,j)-1)+2;
end

%% import node sets

%% Fixed points
fixed=(nodes(nodes(:,2)==-4))';
RestrictionFixed=fixed(fixed(1,:) < (numNode_Block+1));

%% Contact
contact=(nodes(nodes(:,3)==-0.125000000000000))';
Anodes_contact_bodyB=contact(contact(1,:) < (numNode_Block+1));
for j = 1:length(Anodes_contact_bodyB)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyB(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyB(1,j)-1)+2;
end

%% All Nodes
AllNodes=[1:numNode_Block];

%% External

Anodes_ext_loadedB=setdiff(AllNodes,Anodes_contact_bodyB);
for j = 1:length(Anodes_ext_loadedB)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 2.*(Anodes_ext_loadedB(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 2.*(Anodes_ext_loadedB(1,j)-1)+2;
end

%% Finding intersects
if ~isempty(intersect(Anodes_contact_bodyB,RestrictionFixed))
    Contact_Intersect=find(ismember(Anodes_contact_bodyB,RestrictionFixed));
    [Contact_Intersect_DOF]=DOF_Maker(Contact_Intersect);
else
    Contact_Intersect_DOF=[];
end
%
if ~isempty(intersect(Anodes_ext_loadedB,RestrictionFixed))
    Ext_Load_Intersect=find(ismember(Anodes_ext_loadedB,RestrictionFixed));
    [Ext_Load_Intersect_DOF]=DOF_Maker(Ext_Load_Intersect);
else
    Ext_Load_Intersect_DOF=[];
end
%
[internal_Intersect_DOF]=[];

%% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = [];
%% import stiffness matrix
K1 = K(DOF1,DOF1);

%%
[KCB, KEB, AB, BB, CB] = static_reduction_Restricted(K1, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force',...
    internal_Intersect_DOF, Ext_Load_Intersect_DOF,Contact_Intersect_DOF);
new_KC = [ AB BB';BB CB];

%% Updating the nodes base on restrictions
Anodes_contact_bodyB=setdiff(Anodes_contact_bodyB,RestrictionFixed);

save('KCEmilyB.mat');


end