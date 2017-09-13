function [NodeSet, uCTemp, fCTemp, uCTempTime, iStateC, fw]=Solution_Algorithm_Two_Blocks(Anodes_contact_bodyT, Anodes_contact_bodyB, KC, KCT, KCB, KET, KEB, friction_Coefficient)

%Set the initial Values
t=0;
deltaT=0.1;
T=5;
eps0=0.001;

%The number of increment in the time
Number_of_timestep=10;


%Coeff for the pressure load applied
Coeff_Load=1;

%%%
Timestep_counter=1;

Try_Counter=0;

%Solving for each time step
while Timestep_counter < Number_of_timestep 
    if t==0
        for iop=1:length(Anodes_contact_bodyT)
           iStateC(iop,1)=1; 
        end
        
        %Load
        fwt= create_fw_timestep(KET, 'zero', '-constant', Timestep_counter, Number_of_timestep, Coeff_Load);
        fwb= create_fw_timestep(KEB, 'zero', 'constant', Timestep_counter, Number_of_timestep, Coeff_Load);
        fw=create_two_body_fw(KCT, KCB, fwt, fwb);
        
        %Plotting the reaction load fw
        %Plotting some results
        figure;
        for iop=1:length(Anodes_contact_bodyT)
           XP(iop,1)=iop; 
        end
        for iop=1:length(Anodes_contact_bodyT)
           FWpb(iop,1)=fwb(2*iop,1); 
        end
        plot(XP, FWpb,'LineWidth',2);
        
        %Plotting the reaction load fw
        %Plotting some results
        figure;
        for iop=1:length(Anodes_contact_bodyT)
           FWpt(iop,1)=fwt(2*iop,1); 
        end
        plot(XP, FWpt,'LineWidth',2);
        
        %Ploting
        figure;
        for iop=1:length(Anodes_contact_bodyT)
           FWptb(iop,1)=fw(2*iop,1); 
        end
        plot(XP, FWptb,'LineWidth',2);
        
        %Defining initial uC and fC
        [rfw,~]=size(fw);
        
        uCTemp=zeros(rfw,1);
        uCTempTime=zeros(rfw,1);
        fCTemp=fw;
        eps=100;
    else
        
        %Solving for the current time
        [NodeSet, uC, fC]=istate_Solver(Anodes_contact_bodyT, KC, fw, uCTemp, fCTemp, t, friction_Coefficient, iStateC, uCTempTime);
        %Updating uC, fC and using previous istate
        uCTemp=uC;
        fCTemp=fC;
        for iop=1:length(Anodes_contact_bodyT)
           iStateC(iop,1)=NodeSet(iop).istateNumber; 
        end
        
        %Finding the epsilon for the previous tima and current time based
        %on Eq. 16
        if Try_Counter ~= 0
            eps = eps_Compute(Anodes_contact_bodyT ,NodeSetTemp ,NodeSet);
        end
        
        %Extarcting values from previous time to go through current
        %time
        NodeSetTemp=NodeSet;
        %Showing how many tries have been done
        Try_Counter=Try_Counter+1;
        epsCnt(Try_Counter,1)=eps;
    end
    
    %Time increasing
    if eps<eps0 && Try_Counter > 2
        
       
       for iop=1:length(Anodes_contact_bodyT)
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
       Timestep_counter=1+Timestep_counter
       %update for the time
       uCTempTime=uC;
      
       %
       %Increasing in the force
       %Load
       fwt= create_fw_timestep(KET, 'zero', '-constant', Timestep_counter, Number_of_timestep, Coeff_Load);
       fwb= create_fw_timestep(KEB, 'zero', 'constant', Timestep_counter, Number_of_timestep, Coeff_Load);
       fw=create_two_body_fw(KCT, KCB, fwt, fwb);
        
       %Set initial again
        fCTemp=fw;
        Try_Counter=0;
        
       
       %Plotting
       %Plotting some results
        for iop=1:length(Anodes_contact_bodyT)
           XP(iop,1)=iop; 
        end
        for iop=1:length(Anodes_contact_bodyT)
           FWp(iop,1)=fw(2*iop,1); 
        end
        figure;
        xlabel('NodeNumber');
        
        title(['Normal and Tangential Reactions for the ' Timestep_counter 'iteration'])
        hold;
        for iop=1:length(Anodes_contact_bodyT)
           YTP(iop,1)=NodeSet(iop).Tangential_Reaction; 
        end
        plot(XP,YTP,'LineWidth',2);
        for iop=1:length(Anodes_contact_bodyT)
           YP(iop,1)=NodeSet(iop).Normal_Reaction; 
        end
        plot(XP,YP,'LineWidth',2);
        ylabel('Force Value');
        
        figure;
        
        for iop=1:length(Anodes_contact_bodyT)
           YPistate(iop,1)=NodeSet(iop).istateNumber; 
        end
        plot(XP,YPistate,'LineWidth',2);
        title(['iState Condition for the ' Timestep_counter 'iteration']);
        ylabel('iState-Number Condition: 1: Stick, 2: Seperation, 3,4: Slip');
        xlabel('NodeNumber');
        
        
        %displacement for each body
        u1C=uC-KCB\(fC-fwb);
        u2C=uC-u1C;
        
        figure;
        xlabel('NodeNumber');
        
        title(['Normal Displacement for the ','iteration',Timestep_counter,])
        hold;
        for iop=1:length(Anodes_contact_bodyT)
           YTP(iop,1)=u1C(2*iop); 
        end
        plot(XP,YTP,'LineWidth',2);
        
        ylabel('Displacement Value');
        
        figure;
        xlabel('NodeNumber');
        
        title(['Tangential Displacement for the ','iteration',Timestep_counter,])
        hold;
        for iop=1:length(Anodes_contact_bodyT)
           YTP(iop,1)=u2C(2*iop); 
        end
        plot(XP,YTP,'LineWidth',2);
        
        ylabel('Displacement Value');
        
        
    elseif t==0
       t=t+deltaT; 
    end
end

end