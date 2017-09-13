function [fC,uC,istate, CheckLoop]=istate2(KC,fw,uC,fC,i,ff, CheckLoop)

CheckLoop=CheckLoop+1;

p=0;
fC(2*i-1,1)=p;

q=0;
fC(2*i,1)=q;

[rKC,cKC]=size(KC);


%
R1=q-fw(2*i-1,1)-KC(2*i-1,1:2*(i-1))*uC(1:2*(i-1),1)-KC(2*i-1,(2*i+1):cKC)*uC((2*i+1):rKC,1);
R2=p-fw(2*i,1)-KC(2*i,1:2*(i-1))*uC(1:2*(i-1),1)-KC(2*i,(2*i+1):cKC)*uC((2*i+1):rKC,1);
    
    A=KC(2*i-1,2*i-1);
    B=KC(2*i-1,2*i);
    C=KC(2*i,2*i-1);
    D=KC(2*i,2*i);
    
    FT=[R1;R2];
    KT=[A,B;C,D];
    
    XT=KT\FT;
    v2=XT(1,1);
    w2=XT(2,1);
%


uC(2*i-1,1)=v2;
uC(2*i,1)=w2;

if w2 > 0 
    istate=2;
else
    istate=1;
end

end