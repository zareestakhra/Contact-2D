function Solution_Algorithm_NEW_TestEmilly(Anodes_contact_body, Anodes_contact_bodyT,...
    Anodes_contact_bodyB, KC, KET, KEB, KCT, KCB, friction_Coefficient, PreLoadT, PreLoadB, nodes)

%% Set the initial Values
t=0;
deltaT=0.1;
T=5;
eps0=0.0001;

Anodes_contact_bodyTemp=[1:length(Anodes_contact_body)];

%% The number of increment in the time
Number_of_timestep=6;

Timestep_counter=1;
Try_Counter=0;
for iop=1:length(Anodes_contact_body)
    iStateC(iop,1)=1; 
end


%% Load
[fwT, LoadT]= create_fw_timestep(KET, 'zero', '-constant', 6, Number_of_timestep, PreLoadT);
[fwB, LoadB]= create_fw_timestep(KEB, 'zero', '-constant', 6, Number_of_timestep, PreLoadB);

%% Load At Contact
[rfw,~]=size(fwT);
KEFUN=eye(rfw);
fw_ContactT=Load_at_Contact(KEFUN, 'zero', 'constant', 6, Number_of_timestep, 0);
fw_ContactB=Load_at_Contact(KEFUN, 'zero', 'constant', 6, Number_of_timestep, 0);
%fw_Contact=fw_ContactT+fw_ContactB;

%% Load at Contcat Formula
fwT=fwT-fw_ContactT;
fwB=fwB-fw_ContactB;

%% Merging two loads
fw=create_two_body_fw(KCT, KCB, fwT, fwB);
LoadT=LoadT
LoadB=LoadB

%%
[rfw,~]=size(fw);
uCTemp=zeros(rfw,1);
uCTempTime=zeros(rfw,1);
uCTempTimeT=uCTempTime;
uCTempTimeB=uCTempTime;
fCTemp=fw;
eps=100;

%Solving for each time step
while Timestep_counter <= Number_of_timestep 
        
    if Try_Counter == 1
       %Load
        [fwT, LoadT]= create_fw_timestep(KET, 'zero', '-constant', 6, Number_of_timestep, PreLoadT);
        [fwB, LoadB]= create_fw_timestep(KEB, 'zero', '-constant', 6, Number_of_timestep, PreLoadB);
        %% Load at Contcat Formula
        fwT=fwT-fw_ContactT;
        fwB=fwB-fw_ContactB;
        %%
        
        fw=create_two_body_fw(KCT, KCB, fwT, fwB);
        Timestep_counter=Timestep_counter
        LoadT=LoadT
        LoadB=LoadB


    end
        %Solving for the current time
        [NodeSet, uC, fC]=istate_Solver_NEW(Anodes_contact_bodyTemp, KC, fw, uCTemp, fCTemp, t, friction_Coefficient, iStateC, uCTempTime);
        %Updating uC, fC and using previous istate
        uCTemp=uC;
        fCTemp=fC;
        for iop=1:length(Anodes_contact_bodyTemp)
           iStateC(iop,1)=NodeSet(iop).istateNumber; 
        end
        
        %Finding the epsilon for the previous tima and current time based
        %on Eq. 16
        if Try_Counter ~= 0 && Timestep_counter~=0
            eps = eps_Compute(Anodes_contact_bodyTemp ,NodeSetTemp ,NodeSet);
        end
        
        %Extarcting values from previous time to go through current
        %time
        NodeSetTemp=NodeSet;
        %Showing how many tries have been done
        Try_Counter=Try_Counter+1;
        epsCnt(Try_Counter,1)=eps;
    
    %Time increasing
    if eps<eps0 && Try_Counter > 2

       for iop=1:length(Anodes_contact_bodyTemp)
           YPistate(iop,1)=NodeSet(iop).istateNumber; 
        end
       istateCnt(Timestep_counter).i1=sum(YPistate(:)==1);
       istateCnt(Timestep_counter).i2=sum(YPistate(:)==2);
       istateCnt(Timestep_counter).i3=sum(YPistate(:)==3);
       istateCnt(Timestep_counter).i4=sum(YPistate(:)==4);
       %Finding the situation of eps
       epsCnt(epsCnt==1)=[];
       epsCnt(epsCnt==0)=[];
       epsCTNS=sort(epsCnt);
       if length(epsCTNS)>1
            istateCnt(Timestep_counter).i5=epsCTNS(2,1);
            istateCnt(Timestep_counter).i6=epsCTNS(length(epsCTNS)-1,1);
       elseif length(epsCTNS)==1
            istateCnt(Timestep_counter).i5=epsCTNS(1,1);
            istateCnt(Timestep_counter).i6=epsCTNS(length(epsCTNS),1);
       else
           
       end
       %
       %increasing the time
       t=t+deltaT;
       
       toc
       
       %update for the time
       uCTempTime=uC;
       
       % Finding the sorted nodenumber
       coardinates=nodes(Anodes_contact_bodyT, 1:4);
       Cor=sortrows(coardinates,2);
       nodenumbersortAbq=Cor(:,1);
       dlmwrite('NodeNumberSortTop.txt',nodenumbersortAbq','delimiter',',');
       for isort=1:length(Anodes_contact_bodyT)
           SortedNodeNumber(isort,1)=find(Anodes_contact_bodyT==nodenumbersortAbq(isort,1));
       end

       %
       %Increasing in the force
       Timestep_counter=1+Timestep_counter;
       fCTemp=fw;
       Try_Counter=0;
       
       %%
       %Plotting
       %Plotting some results
        for iop=1:length(Anodes_contact_bodyTemp)
           XP(iop,1)=iop; 
        end
        for iop=1:length(Anodes_contact_bodyTemp)
           FWp(iop,1)=fw(2*iop,1); 
        end
        figure;
        xlabel('NodeNumber');
        
        title(['Normal and Tangential Reactions for the ' Timestep_counter 'iteration'])
        
        for iop=1:length(Anodes_contact_bodyTemp)
           YTP(iop,1)=NodeSet(iop).Tangential_Reaction;
        end
        %YTP=YTP(SortedNodeNumber,:);
        plot(XP,YTP,'LineWidth',2);
        ylabel('Shear Force Value');
        %%
        figure;
        for iop=1:length(Anodes_contact_bodyTemp)
           YP(iop,1)=NodeSet(iop).Normal_Reaction;   
        end
        %YP=YP(SortedNodeNumber,:);
        plot(XP,YP,'LineWidth',2);
        ylabel('Normal Force Value');
        
        figure;
        %YPistate=YPistate(SortedNodeNumber,:);
        plot(XP,YPistate,'LineWidth',2);
        title(['iState Condition for the ' Timestep_counter 'iteration']);
        ylabel('iState-Number Condition: 1: Stick, 2: Seperation, 3,4: Slip');
        xlabel('NodeNumber');
        
        %% displacement for each body
        uCT=uC-KCB\(fC-fwB);
        %First Body
        [NodeSetT]=istate_Finder(Anodes_contact_bodyT, uCT, fC, friction_Coefficient, uCTempTimeT);
        
        uCB=uC-uCT;
        [NodeSetB]=istate_Finder(Anodes_contact_bodyB, uCB, fC, friction_Coefficient, uCTempTimeB);
        
        %update for the time
        uCTempTimeT=uCT;
        uCTempTimeB=uCB; 
        
        %% Extarcting values from each body displacement
        for iop=1:length(Anodes_contact_bodyT)
           uHorT(iop,1)=uCT(2*iop-1); 
        end
        for iop=1:length(Anodes_contact_bodyT)
           uVerT(iop,1)=uCT(2*iop); 
        end
        for iop=1:length(Anodes_contact_bodyT)
           uHorB(iop,1)=uCB(2*iop-1); 
        end
        for iop=1:length(Anodes_contact_bodyT)
           uVerB(iop,1)=uCB(2*iop); 
        end
        %% Plotting 
       figure;
       hold;
        xlabel('NodeNumber');
        
        uVerT=uVerT(SortedNodeNumber,:);
        plot(XP,uVerT,'LineWidth',2);
        %plot(XP,uVerB,'LineWidth',2);
        
        ylabel('Vertical Displacement Value');
        

        figure;
        hold;
        xlabel('NodeNumber');
        
        uHorT=uHorT(SortedNodeNumber,:);
        plot(XP,uHorT,'LineWidth',2);
        %plot(XP,uHorB,'LineWidth',2);
        
        ylabel('Horizontal Displacement Value');
        Final_Displacement=[uHorB,uHorT,uVerB,uVerT];
    end

end

end