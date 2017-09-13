function Solution_Algorithm_NEW(Anodes_contact_body, Anodes_contact_bodyT, Anodes_contact_bodyB, KC, KET, KEB, KCT, KCB, friction_Coefficient, PreLoadT, PreLoadB)

%Set the initial Values
t=0;
deltaT=0.1;
T=5;
eps0=0.000001;

%The number of increment in the time
Number_of_timestep=6;

Timestep_counter=1;
Try_Counter=0;
for iop=1:length(Anodes_contact_body)
    iStateC(iop,1)=1; 
end


%Load
[fwT, LoadT]= create_fw_timestep(KET, 'zero', '-constant', 6, Number_of_timestep, PreLoadT);
[fwB, LoadB]= create_fw_timestep(KEB, 'zero', '-constant', 6, Number_of_timestep, PreLoadB);
fw=create_two_body_fw(KCT, KCB, fwT, fwB);
LoadT=LoadT
LoadB=LoadB

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
        fw=create_two_body_fw(KCT, KCB, fwT, fwB);
        Timestep_counter=Timestep_counter
        LoadT=LoadT
        LoadB=LoadB
    end
        %Solving for the current time
        [NodeSet, uC, fC]=istate_Solver(Anodes_contact_body, KC, fw, uCTemp, fCTemp, t, friction_Coefficient, iStateC, uCTempTime);
        %Updating uC, fC and using previous istate
        uCTemp=uC;
        fCTemp=fC;
        for iop=1:length(Anodes_contact_body)
           iStateC(iop,1)=NodeSet(iop).istateNumber; 
        end
        
        %Finding the epsilon for the previous tima and current time based
        %on Eq. 16
        if Try_Counter ~= 0 && Timestep_counter~=0
            eps = eps_Compute(Anodes_contact_body ,NodeSetTemp ,NodeSet);
        end
        
        %Extarcting values from previous time to go through current
        %time
        NodeSetTemp=NodeSet;
        %Showing how many tries have been done
        Try_Counter=Try_Counter+1;
        epsCnt(Try_Counter,1)=eps;
    
    %Time increasing
    if eps<eps0 && Try_Counter > 2
        
       
       for iop=1:length(Anodes_contact_body)
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
      
       %
       %Increasing in the force
       Timestep_counter=1+Timestep_counter;
       fCTemp=fw;
       Try_Counter=0;
       
       %%
       %Plotting
       %Plotting some results
        for iop=1:length(Anodes_contact_body)
           XP(iop,1)=iop; 
        end
        for iop=1:length(Anodes_contact_body)
           FWp(iop,1)=fw(2*iop,1); 
        end
        figure;
        xlabel('NodeNumber');
        
        title(['Normal and Tangential Reactions for the ' Timestep_counter 'iteration'])
        hold;
        for iop=1:length(Anodes_contact_body)
           YTP(iop,1)=NodeSet(iop).Tangential_Reaction; 
        end
        plot(XP,YTP,'LineWidth',2);
        for iop=1:length(Anodes_contact_body)
           YP(iop,1)=NodeSet(iop).Normal_Reaction; 
        end
        plot(XP,YP,'LineWidth',2);
        ylabel('Force Value');
        
        figure;
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
        %%
        figure;
        xlabel('NodeNumber');
        
        title(['Normal Displacement for the ','iteration',Timestep_counter,])
        hold;
        for iop=1:length(Anodes_contact_bodyT)
           YTP(iop,1)=NodeSetT(iop).istateNumber; 
        end
        
        plot(XP,YTP,'LineWidth',2);
        
        ylabel('Displacement Value');
        %%
        figure;
        xlabel('NodeNumber');
        
        title(['Tangential Displacement for the ','iteration',Timestep_counter,])

        for iop=1:length(Anodes_contact_bodyT)
           YTP(iop,1)=NodeSetB(iop).istateNumber; 
        end
        plot(XP,YTP,'LineWidth',2);
        
        ylabel('Displacement Value');
        
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
        %% Plottingg 
       figure;
       hold;
        xlabel('NodeNumber');
        
        
        plot(XP,uVerT,'LineWidth',2);
        plot(XP,uVerB,'LineWidth',2);
        
        ylabel('Vertical Displacement Value');
        

        figure;
        hold;
        xlabel('NodeNumber');
        
        
        plot(XP,uHorT,'LineWidth',2);
        plot(XP,uHorB,'LineWidth',2);
        
        ylabel('Horizontal Displacement Value');
       
    end

end

end