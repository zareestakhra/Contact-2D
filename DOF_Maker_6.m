function [DOFSet]=DOF_Maker_6(nodeset)

    for inode=1:length(nodeset)
        DOFSet(1, 2*(inode-1)+1) = 6.*(nodeset(1,inode)-1)+1;
        DOFSet(1, 2*(inode-1)+2) = 6.*(nodeset(1,inode)-1)+2;
    end

end