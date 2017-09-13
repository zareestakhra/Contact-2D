function [DOFSet]=DOF_Maker(nodeset)

    for inode=1:length(nodeset)
        DOFSet(1, 2*(inode-1)+1) = 2.*(nodeset(1,inode)-1)+1;
        DOFSet(1, 2*(inode-1)+2) = 2.*(nodeset(1,inode)-1)+2;
    end

end