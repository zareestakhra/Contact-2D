function [Anodes_contact_bodyB, KCB, KEB]=get_info_AB_Specific_9_2b()

input_file = '7-17-Bottom-Barber.inp';
mtx_file = '7-17-Bottom-Barber-1_STIF1.mtx';

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
Anodes_contact_bodyB=[10051:10251];

for j = 1:length(Anodes_contact_bodyB)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyB(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyB(1,j)-1)+2;
end


%% Internal
Anodes_internalB=[1:50 , 152:10050];
for j = 1:length(Anodes_internalB)
        Mnodes_internal(1, 2*(j-1)+1) = 2.*(Anodes_internalB(1,j)-1)+1;
        Mnodes_internal(1, 2*(j-1)+2) = 2.*(Anodes_internalB(1,j)-1)+2;
end


%% External
Anodes_ext_loadedB=[51:151];
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
if ~isempty(intersect(Anodes_internalB,RestrictionFixed))
    internal_Intersect=find(ismember(Anodes_internalB,RestrictionFixed));
    [internal_Intersect_DOF]=DOF_Maker(internal_Intersect);
else
    [internal_Intersect_DOF]=[];
end

%% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = Mnodes_internal;
%% import stiffness matrix
K = import_stiffness_matrix(mtx_file);

%%
[KCB, KEB, AB, BB, CB] = static_reduction_Restricted(K, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force',...
    internal_Intersect_DOF, Ext_Load_Intersect_DOF,Contact_Intersect_DOF);
new_KC = [ AB BB';BB CB];

%% Updating the nodes base on restrictions
Anodes_contact_bodyB=setdiff(Anodes_contact_bodyB,RestrictionFixed);

save('KCBot-9-2-CantTotal.mat');
%fw_pressure = resort_load_vector(fw_pressure_unsorted);

end

