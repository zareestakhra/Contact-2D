function writing_tofile_Abq(nodes,RestrictionFixed,charr)

coardinates=nodes(RestrictionFixed, 1:4);
       Cor=sortrows(coardinates,2);
       nodenumbersortAbq=Cor(:,1);
       dlmwrite(charr,nodenumbersortAbq','delimiter',',');

end