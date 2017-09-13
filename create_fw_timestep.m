function [fw_unsorted, Load] = create_fw_timestep( KE, ...
    load_distribution_x, load_distribution_y, timestep_number, Number_of_timesteps, PreLoad)


%============= Create Load Distributions =============%
load_zero = zeros(length(KE(1,:))/2,1);
load_constant = ones(length(KE(1,:))/2,1);
load_linear = zeros(length(KE(1,:))/2,1); %preallocate
for i = 0:length(load_linear)-1
   load_linear(i+1,1) = -1 + 2*(i/(length(load_linear)-1));
end
%========= Select Load Distribution for DOF 1 ========%
if strcmp(load_distribution_x,'zero') == 1;
    load_distribution_x = load_zero;
elseif strcmp(load_distribution_x,'constant') == 1;
    load_distribution_x = load_constant;
elseif strcmp(load_distribution_x,'-constant') == 1;
    load_distribution_x = -load_constant;
elseif strcmp(load_distribution_x,'linear') == 1;
    load_distribution_x = load_linear;
elseif strcmp(load_distribution_x,'-linear') == 1;
    load_distribution_x = -load_linear;
elseif strcmp(load_distribution_x,'pmstep') == 1;
    load_pmstep = [ones(length(KE(1,:))/4,1); ...
        -ones(length(KE(1,:))/4,1)];
    load_distribution_x = load_pmstep;
elseif strcmp(load_distribution_x,'mpstep') == 1;
    load_mpstep = [-ones(length(KE(1,:))/4,1); ...
        ones(length(KE(1,:))/4,1)];
    load_distribution_x = load_mpstep;
else
    error('Error: incorrect load distribution selected for ''y'' direction')
end
%========= Select Load Distribution for DOF 2 ========%
if strcmp(load_distribution_y,'zero') == 1;
    load_distribution_y = load_zero;
elseif strcmp(load_distribution_y,'constant') == 1;
    load_distribution_y = load_constant;
elseif strcmp(load_distribution_y,'-constant') == 1;
    load_distribution_y = -load_constant;
elseif strcmp(load_distribution_y,'linear') == 1;
    load_distribution_y = load_linear;
elseif strcmp(load_distribution_y,'-linear') == 1;
    load_distribution_y = -load_linear;
elseif strcmp(load_distribution_y,'pmstep') == 1;
    load_pmstep = [ones(length(KE(1,:))/4,1); ...
        -ones(length(KE(1,:))/4,1)];
    load_distribution_y = load_pmstep;
elseif strcmp(load_distribution_y,'mpstep') == 1;
    load_mpstep = [-ones(length(KE(1,:))/4,1); ...
        ones(length(KE(1,:))/4,1)];
    load_distribution_y = load_mpstep;
else
    error('Error: incorrect load distribution selected for ''x'' direction')
end
%===================== Create fw =====================%
load_distribution = zeros(2*length(load_distribution_x),1);
for j = 1:(length(KE(1,:))/2)
    load_distribution(2*(j-1)+1, 1) = load_distribution_x(j,1);
    load_distribution(2*(j-1)+2, 1) = load_distribution_y(j,1);
end

%Setting time step for load increment
T=linspace(0,PreLoad, Number_of_timesteps);
load_distribution=T(1,timestep_number)*load_distribution;
Load=T(1,timestep_number);
%load_distribution(2,1)=load_distribution(2,1)/2;
%load_distribution(length(load_distribution),1)=load_distribution(length(load_distribution),1)/2;


fw_unsorted = KE*load_distribution;