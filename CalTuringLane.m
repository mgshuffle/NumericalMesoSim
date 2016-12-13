function turning = CalTuringLane(linkGraph)
[m,~] = size(linkGraph);

[c,r] = find(linkGraph'==2);%consider results of function "find" is order by colIdx. now we want to order by rowIdx. So we do transpose

if isempty(r)
    turning = zeros(m,2); %no lanes to turn
else
    [~,idx1] = unique(r);
    c2 = 2*ones(size(r));
    c2(idx1) = 1;
    turning = sparse(r,c2,c,m,2);
    turning = full(turning);
end

end