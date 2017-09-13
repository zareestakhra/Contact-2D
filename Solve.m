function Solve()
tic;
clear
clc
close all
% define file paths
%load('KCEmilyT.mat')
%load('KCEmilyB.mat')
load('KCEmilyBShortContact.mat')
load('KCEmilyT_Short_Contact.mat')

%Coefficient of Friction
friction_Coefficient=0.6;

%Final Load Value
PreLoadT=1000;
PreLoadB=1000;

%Boundaru condition Nodes
BNNodes=[1];

%Merge two matrices
[KC, ~, ~, ~] = merge_matrices(KCT, KCB);

Anodes_contact_bodyTemp=[1:length(Anodes_contact_bodyT)];

%Solution for the Algorithm
%[NodeSetTemp, uCTemp, fCTemp, uCTempTime, iStateC, fw]=Solution_Algorithm_Two_Blocks(Anodes_contact_bodyT, Anodes_contact_bodyB, KC, KCT, KCB, KET, KEB, friction_Coefficient);

Solution_Algorithm_NEW_TestEmilly(Anodes_contact_bodyTemp, Anodes_contact_bodyT,...
    Anodes_contact_bodyB, KC, KET, KEB, KCT, KCB, friction_Coefficient, PreLoadT, PreLoadB, nodes);
end