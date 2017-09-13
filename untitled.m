clear all;

load('EmilyData.mat');

%% TopBlock

numNode_Block=length(K)/(6*2);

%% DOF2
NodeNumber2=[1:numNode_Block];
for j = 1:length(NodeNumber2)
        DOF2(1, 2*(j-1)+1) = 6.*(NodeNumber2(1,j)-1)+1;
        DOF2(1, 2*(j-1)+2) = 6.*(NodeNumber2(1,j)-1)+2;
end

%% import node sets

%% Fixed points
RestrictionFixed=(nodes(nodes(:,2)==-4 & nodes(:,1)<=2171))';

%% Contact
Anodes_contact_bodyT=(nodes(nodes(:,3)==-0.125000000000000 & nodes(:,2)>=2 & nodes(:,2)<=4 & nodes(:,1)<=2171))';
for j = 1:length(Anodes_contact_bodyT)
        Mnodes_contact_body(1, 2*(j-1)+1) = 6.*(Anodes_contact_bodyT(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 6.*(Anodes_contact_bodyT(1,j)-1)+2;
end

%% External
Anodes_ext_loadedT=(nodes(nodes(:,3)==0.125000000000000 & nodes(:,2) >=2.875 & nodes(:,2) <= 3.125))';
for j = 1:length(Anodes_ext_loadedT)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 6.*(Anodes_ext_loadedT(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 6.*(Anodes_ext_loadedT(1,j)-1)+2;
end

%% All Nodes
AllNodes=[1:numNode_Block];

%% internal

Anodes_internalT=setdiff(AllNodes,union(Anodes_contact_bodyT,Anodes_ext_loadedT));
for j = 1:length(Anodes_internalT)
        Mnodes_internalT(1, 2*(j-1)+1) = 6.*(Anodes_internalT(1,j)-1)+1;
        Mnodes_internalT(1, 2*(j-1)+2) = 6.*(Anodes_internalT(1,j)-1)+2;
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
%
if ~isempty(intersect(Anodes_internalT,RestrictionFixed))
    internal_Intersect=find(ismember(Anodes_internalT,RestrictionFixed));
    [internal_Intersect_DOF]=DOF_Maker(internal_Intersect);
else
    [internal_Intersect_DOF]=[];
end

%% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = Mnodes_internal;
%% import stiffness matrix
%K2 = K(DOF2,DOF2


load('EmilyData.mat');

%% TopBlock

numNode_Block=length(K)/(6*2);

%% DOF2
NodeNumber2=[numNode_Block+1:length(K)/6];
for j = 1:length(NodeNumber2)
        DOF2(1, 2*(j-1)+1) = 6.*(NodeNumber2(1,j)-1)+1;
        DOF2(1, 2*(j-1)+2) = 6.*(NodeNumber2(1,j)-1)+2;
end

%% import node sets

%% Fixed points
RestrictionFixed=(nodes(nodes(:,2)==-4 & nodes(:,1)>2171))';

%% Contact
Anodes_contact_bodyB=(nodes(nodes(:,3)==-0.125000000000000 & nodes(:,2)>=2 & nodes(:,2)<=4 & nodes(:,1)>2171))';

for j = 1:length(Anodes_contact_bodyB)
        Mnodes_contact_body(1, 2*(j-1)+1) = 6.*(Anodes_contact_bodyB(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 6.*(Anodes_contact_bodyB(1,j)-1)+2;
end

%% External
Anodes_ext_loadedB=(nodes(nodes(:,3)==-0.375 & nodes(:,2) >=2.875 & nodes(:,2) <= 3.125))';
for j = 1:length(Anodes_ext_loadedB)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 6.*(Anodes_ext_loadedB(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 6.*(Anodes_ext_loadedB(1,j)-1)+2;
end

%% All Nodes
AllNodes=[numNode_Block+1:length(K)/6];

%% internal
Anodes_internalB=setdiff(AllNodes,union(Anodes_contact_bodyB,Anodes_ext_loadedB));
for j = 1:length(Anodes_internalB)
        Mnodes_internalB(1, 2*(j-1)+1) = 6.*(Anodes_internalB(1,j)-1)+1;
        Mnodes_internalB(1, 2*(j-1)+2) = 6.*(Anodes_internalB(1,j)-1)+2;
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
if ~isempty(intersect(Anodes_internalB,RestrictionFixed))
    internal_Intersect=find(ismember(Anodes_internalB,RestrictionFixed));
    [internal_Intersect_DOF]=DOF_Maker(internal_Intersect);
else
    [internal_Intersect_DOF]=[];
end

%% combine node sets
contact_Mnodes = Mnodes_contact_body-2*numNode_Block;
ext_loaded_Mnodes = Mnodes_ext_loaded-2*numNode_Block;
internal_Mnodes = Mnodes_internal-2*numNode_Block;
%% import stiffness matrix
%K2 = K(DOF2,DOF2);
