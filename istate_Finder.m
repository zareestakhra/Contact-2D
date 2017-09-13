function [NodeSet]=istate_Finder(Anodes_contact_body, uC, fC, ff, uCTempTime)
%Finding the number of the nodes at the contact surface
[~,rAc]=size(Anodes_contact_body);

for i=1:rAc
    
    if fC(2*i,1)>0 && -1*ff*fC(2*i,1)< fC(2*i-1,1) <ff*fC(2*i,1) ...
        istate=1;
    elseif uC(2*i-1,1)-uCTempTime(2*i-1,1)>0 ...
            && fC(2*i,1)>0 
        istate=3;
    elseif uC(2*i-1,1)-uCTempTime(2*i-1,1)<0 ...
            && fC(2*i,1)>0 
        istate=4;
    else
        istate=2;
    end
    
%Put the information for each node in a structure
NodeSet(i).istateNumber=istate;

end

end