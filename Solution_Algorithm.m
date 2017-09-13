function [NodeSet, uC]=Solution_Algorithm(Anodes_contact_body, KC, friction_Coefficient, uCTemp, fCTemp ,fw ,uCTempTime)

%Set the initial Values
t=1;
eps0=0.001;
eps=100;
Timestep_counter=1;

%Coeff for the pressure load applied
Coeff_Load=1;

%%%
Try_Counter=0;

for iop=1:length(Anodes_contact_body)
    iStateC(iop,1)=1; 
end

%Solving
while eps>eps0  
    
        %Solving for the current time
        [NodeSet, uC, fC]=istate_Solver(Anodes_contact_body, KC, fw, uCTemp, ...
        fCTemp, t, friction_Coefficient, iStateC, uCTempTime);
        
        %Updating uC, fC and using previous istate
        uCTemp=uC;
        fCTemp=fC;
        for iop=1:length(Anodes_contact_body)
           iStateC(iop,1)=NodeSet(iop).istateNumber; 
        end
        
        %Finding the epsilon for the previous tima and current time based
        %on Eq. 16
        if Try_Counter ~= 0
            eps = eps_Compute(Anodes_contact_body ,NodeSetTemp ,NodeSet);
        end
        
        %Extarcting values from previous time to go through current
        %time
        NodeSetTemp=NodeSet;
        %Showing how many tries have been done
        Try_Counter=Try_Counter+1;
        epsCnt(Try_Counter,1)=eps;
end      
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
       end
       toc
end
     
