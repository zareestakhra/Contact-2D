function [DOFSet]=DOF_MakerTtranspose(nodeset)

    for inode=1:size(nodeset)
        DOFSet(2*(inode-1)+1,1) = 2.*(nodeset(inode,1)-1)+1;
        DOFSet(2*(inode-1)+2,1) = 2.*(nodeset(inode,1)-1)+2;
    end

end