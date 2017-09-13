function [istateFinal, uC, uCT, uCB, fC,fCT, fCB, istateT, istateB]=istate_Check(istate, istateT, istateB , uC, uCT, uCB, fC, fCT, fCB, i)

%%If they have the same istate
if istateT==istateB && (istateT ~=2)
    %% for the Stick and Slipping
    if fC(2*i-1,1) == fCT(2*i-1,1) && ...
            fC(2*i,1) == fCT(2*i,1) && fC(2*i-1,1) == fCB(2*i-1,1)...
            && fC(2*i,1) == fCB(2*i,1) && ...
            uC(2*i-1,1)==uCT(2*i-1,1)+uCB(2*i-1,1)...
            && uC(2*i,1)==uCT(2*i,1)+uCB(2*i,1)...
            && uC(2*i,1)=0; 
        istateFinal=istateT;
        %Update fC & uC
        fC(2*i,1) = fCT(2*i,1); 
        fC(2*i-1,1) = fCB(2*i-1,1);
        uC(2*i-1,1)=uCT(2*i-1,1)+uCB(2*i-1,1);
        uC(2*i,1)=uCT(2*i,1)+uCB(2*i,1);
    end
    
elseif istateT==istateB && (istateT ==2) 
    %% for the Seperation
    if istateT==2 && 0 == fCT(2*i-1,1) && ...
            0 == fCT(2*i,1) && 0 == fCB(2*i-1,1)...
            && 0 == fCB(2*i,1) && ...
            uC(2*i-1,1)==uCT(2*i-1,1)+uCB(2*i-1,1)...
            && uC(2*i,1)==uCT(2*i,1)+uCB(2*i,1)
        istateFinal=istateT;
        %Update fC & uC
        fC(2*i,1) = fCT(2*i,1); 
        fC(2*i-1,1) = fCB(2*i-1,1);
        uC(2*i-1,1)=uCT(2*i-1,1)+uCB(2*i-1,1);
        uC(2*i,1)=uCT(2*i,1)+uCB(2*i,1);
    end
end