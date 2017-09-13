function [Anodes_contact_bodyB, KCT, KET]=get_info_Bot_Without_Internal()

input_file = '7-17-Bottom-Barber.inp';
mtx_file = '7-17-Bottom-Barber-1_STIF1.mtx';

lengthFile=find_input_file_length(input_file);

%% Boundary Condition nodes
RestrictionFixed=[];
iBound=1;
nnBound=1;

while iBound <= 51
    
RestrictionFixed=[RestrictionFixed, nnBound];

iBound=iBound+1;
nnBound=nnBound+201;

end

%%
Anodes_contact_bodyB=[10051:10251];
for j = 1:length(Anodes_contact_bodyB)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyB(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyB(1,j)-1)+2;
end

%% All Nodes
AllNodes=[1:10251];

%%
Anodes_ext_loadedB=setdiff(AllNodes,union(Anodes_contact_bodyB,RestrictionFixed));
%Anodes_ext_loadedB=[10051:10251];
for j = 1:length(Anodes_ext_loadedB)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 2.*(Anodes_ext_loadedB(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 2.*(Anodes_ext_loadedB(1,j)-1)+2;
end



% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = [];
% import stiffness matrix
K = import_stiffness_matrix(mtx_file);

%% for Boundary condition setting

if ~isempty(RestrictionFixed)
for i=1:length(RestrictionFixed)
    R=RestrictionFixed(i);
    K(2*R-1,:)=0;
    K(:,2*R-1)=0;
    K(2*R-1, 2*R-1)=1;
    K(2*R,:)=0;
    K(:,2*R)=0;
    K(2*R, 2*R)=1;
end
end
%%

[KCB, KEB, AB, BB, CB] = static_reduction(K, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force');
new_KC = [ AB BB';BB CB];


save('KCBotttom-8-27-without-Internal.mat');
%fw_pressure = resort_load_vector(fw_pressure_unsorted);

end