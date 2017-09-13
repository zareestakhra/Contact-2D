function [Anodes_contact_bodyT, KCT, KET]=get_info_AB_Specific_8_27t()

input_file = '7-17-Top-Barber.inp';
mtx_file = '7-17-Top-Barber-1_STIF1.mtx';

lengthFile=find_input_file_length(input_file);
% import node sets


Anodes_contact_bodyT=[1:201];
for j = 1:length(Anodes_contact_bodyT)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_bodyT(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_bodyT(1,j)-1)+2;
end



Anodes_internalT=[202:10100 , 10202:10251];
%Anodes_internalT=[202:10050];
for j = 1:length(Anodes_internalT)
        Mnodes_internal(1, 2*(j-1)+1) = 2.*(Anodes_internalT(1,j)-1)+1;
        Mnodes_internal(1, 2*(j-1)+2) = 2.*(Anodes_internalT(1,j)-1)+2;
end



Anodes_ext_loadedT=[10101:10201];
%Anodes_ext_loadedT=[10051:10251];
for j = 1:length(Anodes_ext_loadedT)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 2.*(Anodes_ext_loadedT(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 2.*(Anodes_ext_loadedT(1,j)-1)+2;
end



% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = Mnodes_internal;
% import stiffness matrix
K = import_stiffness_matrix(mtx_file);

%% for singularity
RestrictionFixed=[];
iBound=1;
nnBound=1;

while iBound <= 51
    
RestrictionFixed=[RestrictionFixed, nnBound];

iBound=iBound+1;
nnBound=nnBound+201;

end

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

[KCT, KET, AT, BT, CT] = static_reduction(K, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force');
new_KCT = [ AT BT';BT CT];


save('KCTop-8-27-CantTotal2.mat');
%fw_pressure = resort_load_vector(fw_pressure_unsorted);

end