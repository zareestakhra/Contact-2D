function [NodeSet, uC, fC, istateTemp]=istate_Solver_NEW(Anodes_contact_body, KC, fw, uC, fC, t, ff, iStateC, uCTempTime)
uCTemp=uC;
%Finding the number of the nodes at the contact surface
[~,rAc]=size(Anodes_contact_body);
Remaining_Points=[];
for i=1:rAc
istate=iStateC(i,1);
%to check when the loop should terminated in while
CheckNumber=0;
CheckLoop=0;

while (CheckNumber==0)
    if istate==1
        [fC,uC,istateTemp, CheckLoop]=istate1(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop);
        %if the first stimate for istate is correct
        if istate==istateTemp
            CheckNumber=1;
        %if the first stimate for istate is not correct
        else
            if CheckLoop>2
                CheckNumber=1;
                istate=iStateC(i,1);
                Remaining_Points=[Remaining_Points,i];
            else
                istate=istateTemp;
            end
        end
    elseif istate==2
        [fC,uC,istateTemp, CheckLoop]=istate2(KC,fw,uC,fC,i,ff, CheckLoop);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop>2
                CheckNumber=1;
                istate=iStateC(i,1);
                Remaining_Points=[Remaining_Points,i];
            else
                istate=istateTemp;
            end
        end
    elseif istate==3
        [fC,uC,istateTemp, CheckLoop]=istate3(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop>2
                CheckNumber=1;
                istate=iStateC(i,1);
                Remaining_Points=[Remaining_Points,i];
            else
                istate=istateTemp;
            end
        end
    elseif istate==4
        [fC,uC,istateTemp, CheckLoop]=istate4(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop);
        if istate==istateTemp
            CheckNumber=1;
        else
            if CheckLoop>2
                CheckNumber=1;
                istate=iStateC(i,1);
                Remaining_Points=[Remaining_Points,i];
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

%if ~isempty(Remaining_Points)
%    iStateCR(1:length(Remaining_Points))=1;
%    [NodeSet, uC, fC]=istate_Solver_R(Remaining_Points, KC, fw, uC, fC, t, ff, iStateCR, uCTempTime, NodeSet);
%end

end