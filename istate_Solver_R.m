function [NodeSet, uC, fC, istateTemp]=istate_Solver_R(Anodes_contact_body, KC, fw, uC, fC, t, ff, iStateC, uCTempTime, NodeSet)
uCTemp=uC;
%Finding the number of the nodes at the contact surface
[~,rAc]=size(Anodes_contact_body);

for i=1:rAc
istate=iStateC(i,1);
%to check when the loop should terminated in while
CheckNumber=0;
CheckLoop1=0;
CheckLoop2=0;
CheckLoop3=0;
CheckLoop4=0;
while (CheckNumber==0)
    if istate==1
        [fC,uC,istateTemp, CheckLoop1]=istate1(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop1);
        %if the first stimate for istate is correct
        if istate==istateTemp
            CheckNumber=1;
        %if the first stimate for istate is not correct
        else
            if CheckLoop1>3
                CheckNumber=1;
                %istate=iStateC(i,1);
            else
                istate=istateTemp;
            end
        end
    elseif istate==2
        [fC,uC,istateTemp, CheckLoop2]=istate2(KC,fw,uC,fC,i,ff, CheckLoop2);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop2>3
                CheckNumber=1;
                %istate=iStateC(i,1);
            else
                istate=istateTemp;
            end
        end
    elseif istate==3
        [fC,uC,istateTemp, CheckLoop3]=istate3(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop3);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop3>3
                CheckNumber=1;
                %istate=iStateC(i,1);
            else
                istate=istateTemp;
            end
        end
    elseif istate==4
        [fC,uC,istateTemp, CheckLoop4]=istate4(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop4);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop4>3
                CheckNumber=1;
                %istate=iStateC(i,1);
            else
                istate=istateTemp;
            end
        end
    end
end

%Put the information for each node in a structure
NodeSet(i).time = t;
NodeSet(i).Normal_Reaction=fC(2*i,1);
NodeSet(i).Tangential_Reaction=fC(2*i-1,1);
NodeSet(i).Normal_Displacement=uC(2*i,1);
NodeSet(i).Tangential_Displacement=uC(2*i-1,1);
NodeSet(i).istateNumber=istate;
%NodeSet(i).ContactForceVector=fC;
%NodeSet(i).ContactDisplacmentVector=uC;

end


end