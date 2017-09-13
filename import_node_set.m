function [matlab_node_set, abaqus_node_set] = ...
    import_node_set(input_file, nodeset_name, ... 
    input_file_length, DOF_to_import, sorting_DOF, sorting_order)
% 
% [matlab_node_set, abaqus_node_set] = import_node_set(input_file, ...
% nodeset_name, input_file_length, DOF_to_import, sorting_DOF,...
% sorting_order) is a function that imports node sets from a .inp file,
% i.e. an ABAQUS/CAE input file. This function has several options.
% 
% INPUTS:   input_file: the path to the ABAQUS .inp file, i.e. 'example.inp'
%           nodeset_name: the name of the ABAQUS node set to be imported,
% i.e. 'my node set'
%           input_file_length: this is the number of lines contained in the
% input file from which the node sets are to be imported. A function called
% 'find_input_file_length' exists and can be used to determine this value.
%           DOF_to_import: specify the degrees of freedom to import. The
% options for this input are 'x' (imports only degrees of freedom in the x
% direction), 'y' (imports only degrees of freedom in the y direction), or 
% 'both' (imports both degrees of freedom from each node). If this field 
% can be left blank, then 'both' degrees of are imported from each node.
% Note that this field cannot be left blank sorting is required.
%           sorting_DOF: if the order of the nodes in the node set is
% important, this function can sort the nodes according to their position
% in the finite element model. If sorting is not required, then leave this
% and the next input field blank and only input the first 3 (or 4) input
% values. If sorting is required the options are 'x' or 'y'. This specifies
% which coordinate direction the nodes are to be sorted with respect to.
%           sorting_order: this specifies whether the nodes are to be
% sorted in 'ascending' or 'descending' order. The options are 'ascending'
% and 'descending'.
% 
% OUTPUTS:  matlab_node_set: this outputs the degrees of freedom according
% to the numbering convention that is adopted for the static reduction
% procedure in MATLAB.
%           abaqus_node_set: this outputs the abaqus node numbers of nodes
% contained in the node set. This output is not required.
% 
% EXAMPLES: Import both degrees of freedom of 'my node set' from the
% input file 'example.inp' and save both the MATLAB node numbers and the
% abaqus node numbers:
% [matlab_node_set, abaqus_node_set] = ...
%     import_node_set('example.inp', 'my node set', input_file_length);
%           Import only the 'x' direction degrees of freedom of 
% 'my node set' from the input file 'example.inp' and sort the nodes
% according to their 'y' position coordinate and save both the MATLAB 
% and abaqus node numbers:
% [matlab_node_set, abaqus_node_set] = ...
%     import_node_set('example.inp', 'my node set', input_file_length, ...
%     'x', 'y', 'ascending');

%================== Import Node Set ==================%
% find line that contains the name of the node set to be imported
find_line = ['*Nset, nset=',nodeset_name];
i = 0; count = 0;
read_input_file = fopen(input_file,'r');
while i == 0
    count = count +1;
    current_line = fgetl(read_input_file);
    i = strcmp(current_line,find_line);
    if count > input_file_length, break, end
end
% if found, import the node set
if count < input_file_length
    tempvar = textscan(read_input_file,'%n','delimiter',',');
    node_set = tempvar{1}; 
    fclose(read_input_file);
% if not found see if the command ', generate' is after the node set
elseif count > input_file_length
    i = 0; count =0; fclose(read_input_file);
    read_input_file = fopen(input_file,'r');    
    find_line = ['*Nset, nset=',nodeset_name,', generate'];
    while i == 0
        count = count +1;
        current_line = fgetl(read_input_file);
        i = strcmp(current_line,find_line);
        if count > input_file_length, break, end
    end
%     if still not found generate an error message
    if count > input_file_length
        error('Error: cannot find nodeset in input file.')
    end
%     if found once ', generate' is added, import the node set
    temp_textscan_output = textscan(read_input_file,'%d',...
        'delimiter',',');
    node_info = temp_textscan_output{1}';
    node_set = zeros(1,1+(node_info(1,2)-...
        node_info(1,1))/node_info(1,3));
    for i = 1:length(node_set)
        node_set(1,i) = node_info(1,1) + ...
            (i-1)*(node_info(1,3));
    end
    node_set = node_set';
    fclose(read_input_file);
end
%=================== Sort Node Set ===================%
% check if sorting inputs exist, and sort the node set if they do
if exist('sorting_DOF','var') == 1
%     find line where node number and position coordinates are listed
    find_line = '*Node'; 
    i = 0; count = 0;
    read_input_file = fopen(input_file,'r');
    while i == 0
        count = count +1;
        current_line = fgetl(read_input_file);
        i = strcmp(current_line,find_line);
        if count > input_file_length, break, end
    end
    if count > input_file_length
        error('Error: cannot find line ''*Node'' in input file.')
    end  
%     extract numbers and posistions of all nodes in the input file
    tempvar = textscan(read_input_file,'%n, %n, %n');
    all_nodes = tempvar{1};
    node_positions = [tempvar{2},tempvar{3}];
%     extract position coordinates of only the nodes in the node set
    info_list = zeros(length(node_set),3);
    j = 1;
    for i = 1:node_set(length(node_set))
        if node_set(j,1) == all_nodes(i);
            info_list(j,:) = [i,node_positions(i,:)];
            j=j+1;
        end
    end
%     define which coordinate direction to sort with respect to
    if strcmp(sorting_DOF,'x') == 1
        sort_column = 2;
    elseif strcmp(sorting_DOF,'y') == 1
        sort_column = 3;
    else
        error('Error: incorrect input option selected for ''sorting_DOF''')
    end
%     sort the node set in ascending or descending order
    if strcmp(sorting_order,'ascending')
        ordered_info_list = sortrows(info_list,sort_column);
        abaqus_node_set = ordered_info_list(:,1)';
    elseif strcmp(sorting_order,'descending')
        ordered_info_list = sortrows(info_list,-sort_column);
        abaqus_node_set = ordered_info_list(:,1)';
    else
        error('Error: incorrect input option selected for ''sorting_order''')
    end
% check if sorting inputs exist, and don't sort the node set if they don't
elseif exist('sorting_DOF','var') == 0
    abaqus_node_set = node_set';
else
    error('Error: error trying to sort the node set')
end
%============ Convert A-Nodes to M-nodes =============%
if exist('DOF_to_import','var')==0
    DOF_to_import = 'both';
end
if strcmp(DOF_to_import,'x') == 1
    matlab_node_set = zeros(1,length(abaqus_node_set));
    for j = 1:length(abaqus_node_set)
        matlab_node_set(1,j) = 2.*(abaqus_node_set(1,j)-1)+1;
    end
elseif strcmp(DOF_to_import,'y') == 1
    matlab_node_set = zeros(1,length(abaqus_node_set));
    for j = 1:length(abaqus_node_set)
        matlab_node_set(1,j) = 2.*(abaqus_node_set(1,j)-1)+2;
    end
elseif strcmp(DOF_to_import,'both') == 1
    matlab_node_set = zeros(1,2*length(abaqus_node_set));
    for j = 1:length(abaqus_node_set)
        matlab_node_set(1, 2*(j-1)+1) = 2.*(abaqus_node_set(1,j)-1)+1;
        matlab_node_set(1, 2*(j-1)+2) = 2.*(abaqus_node_set(1,j)-1)+2;
    end
else
    error('Error: incorrect input option selected ''DOF_to_import''')
end
