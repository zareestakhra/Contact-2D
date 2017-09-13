function[input_file_length] = find_input_file_length(input_file)
% 
% input_file_length = find_input_file_length(input_file_path) is a 
% function that counts the number of lines in an ABAQUS/CAE input (.inp) 
% file.
% 
% INPUT:    input_file: is the file path of the ABAQUS input file. The path
% is most easily written if the input file is in MATLAB's working
% directory, in which case it can be written, e.g. 'example.inp'.
% 
% OUTPUT:   input_file_length: is a single scalar value of the number of
% lines in the input file.
% 
% EXAMPLE: input_file_length = find_input_file_length('example.inp');

%============== Find Input File Length ===============%
% reads one character of each line in the input file and skips to the next
% line, counting the number of steps.
input_file_length = numel(textread(input_file,'%1c%*[^\n]'));