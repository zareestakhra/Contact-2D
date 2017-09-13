function eps= eps_Compute(Anodes_contact_body ,nodeSet1 ,nodeSet2)

%Finding the number of the nodes at the contact surface
[~,rAc]=size(Anodes_contact_body);

%Finding DeltaV and DeltaW for all nodes for the time two adjacent time
%steps

deltaW(1)=0;
deltaV(1)=0;

for i=1:rAc
    if abs(nodeSet2(i).Normal_Displacement) ~= 0
        deltaW(i)=(abs(nodeSet2(i).Normal_Displacement)-abs(nodeSet1(i).Normal_Displacement))/(abs(nodeSet2(i).Normal_Displacement));
    end
    if abs(nodeSet2(i).Tangential_Displacement) ~= 0
        deltaV(i)=(abs(nodeSet2(i).Tangential_Displacement)-abs(nodeSet1(i).Tangential_Displacement))/(abs(nodeSet2(i).Tangential_Displacement));
    end
end

%Finding maximum value according to Eq. 16
 eps= max(max(deltaV) , max(deltaW));

end

