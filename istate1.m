function [fC,uC,istate, CheckLoop]=istate1(KC,fw,uC,fC,i,ff,uCTemp, uCTempTime, CheckLoop)

CheckLoop=CheckLoop+1;

uC(2*i,1)=0;
uC(2*i-1,1)=uCTempTime(2*i-1,1);


fCTemp=KC*uC+fw;

q=fCTemp(2*i-1,1);
p=fCTemp(2*i,1);

fC(2*i-1,1)=q;
fC(2*i,1)=p;

if p <= 0 
    istate=2;
elseif q >= ff*p
    istate=4;
elseif q <= -1*ff*p
    istate=3;
else
    istate=1;
end

end
