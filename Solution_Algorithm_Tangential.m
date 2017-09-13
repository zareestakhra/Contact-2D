function Solution_Algorithm_Unloading(Anodes_contact_body, KC, KE, uCTemp, fCTemp, friction_Coefficient, iStateC, uCTempTime, NodeSetTemp, fwTemp)

%Set the initial Values
t=0;
deltaT=0.1;

eps0=0.001;

%Coefficient for tangential load
 Coeff_Load=1;

%For testing the range of stick and slip
Coeff=1;

%The number of increment in the time
Number_of_timestep=10;



Timestep_counter=1;

Try_Counter=0;

%Initial Values
fw = create_fw_timestep_unloading(KE, 'zero', '-constant', Timestep_counter, Number_of_timestep);

%Defining initial uC and fC
fCTemp=fw;

%Solving for each time step
while Timestep_counter<Number_of_timestep        
        
        %Defining the load
        

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
        eps = eps_Compute(Anodes_contact_body ,NodeSetTemp ,NodeSet);

        %Extarcting values from previous time to go through current
        %time
        NodeSetTemp=NodeSet;
        
        %Showing how many tries have been done
        Try_Counter=Try_Counter+1;
        epsCnt(Try_Counter,1)=eps;

    
    %Time increasing
    if eps<eps0 
        
       %Getting information about the results
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

       %increasing the time
       t=t+deltaT;
       
       %Claculating the time
       toc
       %Going to the next iteration
       Timestep_counter=1+Timestep_counter
       
       %update for the time
       uCTempTime=uC;
      
       %Increasment in the force
        fw = create_fw_timestep_unloading(KE, 'zero', '-constant', Timestep_counter, Number_of_timestep);
 
        %Updating the force for the next iteration
        fCTemp=fw;
        
        Try_Counter=0;
    end
    
end

end
