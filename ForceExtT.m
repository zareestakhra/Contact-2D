function [fwT]=ForceExtT()

clear all;

load('EmilyData.mat');

%% TopBlock
numNode_Block=length(K)/(6*2);

%% import node sets

%% Fixed points
fixed=(nodes(nodes(:,2)==-4))';
RestrictionFixed=fixed(fixed(1,:) > (numNode_Block));

%% Contact
contact=(nodes(nodes(:,3)==-0.125000000000000))';
Anodes_contact_bodyT=contact(contact(1,:) > numNode_Block);
for j = 1:length(Anodes_contact_bodyT)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyT(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyT(1,j)-1)+2;
end

%% All Nodes
AllNodes=[numNode_Block+1:length(K)/6];

%% External
Anodes_ext_loadedT=setdiff(AllNodes,Anodes_contact_bodyT);
for j = 1:length(Anodes_ext_loadedT)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 6.*(Anodes_ext_loadedT(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 6.*(Anodes_ext_loadedT(1,j)-1)+2;
end

%% Finding intersects
if ~isempty(intersect(Anodes_contact_bodyT,RestrictionFixed))
    Contact_Intersect=find(ismember(Anodes_contact_bodyT,RestrictionFixed));
    [Contact_Intersect_DOF]=DOF_Maker(Contact_Intersect);
else
    Contact_Intersect_DOF=[];
end
%
if ~isempty(intersect(Anodes_ext_loadedT,RestrictionFixed))
    Ext_Load_Intersect=find(ismember(Anodes_ext_loadedT,RestrictionFixed));
    [Ext_Load_Intersect_DOF]=DOF_Maker(Ext_Load_Intersect);
else
    Ext_Load_Intersect_DOF=[];
end

%% combine node sets
ext_loaded_Mnodes = Mnodes_ext_loaded;

%% import Load matrix
fwT = Loading(Mnodes_ext_loaded,1);

%
if ~isempty(Ext_Load_Intersect)
    fwT(Ext_Load_Intersect_DOF,:)=[];
end
%
if ~isempty(Contact_Intersect_DOF)
    fwT(Contact_Intersect_DOF,:)=[];
end


end