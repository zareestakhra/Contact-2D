function [fC,uC,istate, CheckLoop]=istate3(KC,fw,uC,fC,i,ff,uCTemp,uCTempTime, CheckLoop)

CheckLoop=CheckLoop+1;

w=0;
uC(2*i,1)=0;
v=uCTempTime(2*i-1,1);

p=fC(2*i,1);
q=-1*ff*p;
fC(2*i-1,1)=q;

[rKC,cKC]=size(KC);
v2=((q-fw(2*i-1,1))-KC(2*i-1,1:2*(i-1))*uC(1:2*(i-1),1)-KC(2*i-1,2*i)*w-KC(2*i-1,(2*i+1):cKC)*uC((2*i+1):rKC,1))/(KC(2*i-1,2*i-1));
uC(2*i-1,1)=v2; 

p=KC(2*i,:)*uC(:,1)+fw(2*i,1);
fC(2*i,1)=p;

if p <= 0 
    istate=2;
elseif (v2-v)<= 0
    istate=1;
else
    istate=3;
end

end