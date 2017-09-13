function [Anodes_contact_body, KC, KE]=get_info_AB_Specific_4_25()

input_file = '7-6-Block-Bot.inp';
mtx_file = '7-6-Block-Bot-1_STIF1.mtx';

lengthFile=find_input_file_length(input_file);
% import node sets


Anodes_contact_body=[10051:10251];
for j = 1:length(Anodes_contact_body)
        Mnodes_contact_body(1, 2*(j-1)+1) = 2.*(Anodes_contact_body(1,j)-1)+1;
        Mnodes_contact_body(1, 2*(j-1)+2) = 2.*(Anodes_contact_body(1,j)-1)+2;
end



Anodes_internal=[1:50 , 152:10050];
for j = 1:length(Anodes_internal)
        Mnodes_internal(1, 2*(j-1)+1) = 2.*(Anodes_internal(1,j)-1)+1;
        Mnodes_internal(1, 2*(j-1)+2) = 2.*(Anodes_internal(1,j)-1)+2;
end



Anodes_ext_loaded=[51:151];
for j = 1:length(Anodes_ext_loaded)
        Mnodes_ext_loaded(1, 2*(j-1)+1) = 2.*(Anodes_ext_loaded(1,j)-1)+1;
        Mnodes_ext_loaded(1, 2*(j-1)+2) = 2.*(Anodes_ext_loaded(1,j)-1)+2;
end



% combine node sets
contact_Mnodes = Mnodes_contact_body;
ext_loaded_Mnodes = Mnodes_ext_loaded;
internal_Mnodes = Mnodes_internal;
% import stiffness matrix
K = import_stiffness_matrix(mtx_file);

[KC, KE, A, B, C] = static_reduction(K, internal_Mnodes, ext_loaded_Mnodes, contact_Mnodes, 'force');
new_KC = [ A B';B C];


save('KC7-6-Bot.mat');
%fw_pressure = resort_load_vector(fw_pressure_unsorted);

end