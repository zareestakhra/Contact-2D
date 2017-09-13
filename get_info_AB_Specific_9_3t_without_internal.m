function [Anodes_contact_bodyT, KCT, KET]=get_info_AB_Specific_9_3t_without_internal()

input_file = '7-17-Top-Barber.inp';
mtx_file = '7-17-Top-Barber-1_STIF1.mtx';

lengthFile=find_input_file_length(input_file);
% import node sets
%% Fixed points
RestrictionFixed=[];
iBound=1;
nnBound=1;

while iBound <= 51
    
RestrictionFixed=[RestrictionFixed, nnBound];

iBound=iBound+1;
nnBound=nnBound+201;

end

%% Contact
Anodes_contact_bodyT=[1:201];
for j = 1:length(Anodes_contact_bodyT)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyT(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyT(1,j)-1)+2;
end

%% All Nodes
AllNodes=[1:10251];

%% External

Anodes_ext_loadedT=setdiff(AllNodes,Anodes_contact_bodyT);
for j = 1:length(Anodes_ext_loadedT)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 2.*(Anodes_ext_loadedT(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 2.*(Anodes_ext_loadedT(1,j)-1)+2;
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
[internal_Intersect_DOF]=[];

%% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = [];
%% import stiffness matrix
K = import_stiffness_matrix(mtx_file);

%%
[KCT, KET, AT, BT, CT] = static_reduction_Restricted(K, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force',...
    internal_Intersect_DOF, Ext_Load_Intersect_DOF,Contact_Intersect_DOF);
new_KCT = [ AT BT';BT CT];

%% Updating the nodes base on restrictions
Anodes_contact_bodyT=setdiff(Anodes_contact_bodyT,RestrictionFixed);

save('KCTop-9-3-without-internal.mat');
%fw_pressure = resort_load_vector(fw_pressure_unsorted);

end

