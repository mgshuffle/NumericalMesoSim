function [emitTable] = emitTableGeneraterR(ODList,vlim,laneCount,simStep)
emitlaneID = (1:laneCount)';

emitTable = [];
%ODIdx = ODList(1);
demand = ODList(:,2);
fTime = ODList(:,3);
tTime = ODList(:,4);

m = floor((tTime-fTime)/simStep);
p = demand ./ m;

for i = 1:length(demand)
    R = rand(m(i),1);
    idx = find(R<=p(i));
    n = length(idx);
    
    time_ = idx*simStep + fTime(i);
    vel_ = min(vlim(2),max(vlim(1), randn(n,1)*2+9 ));
    laneIdx = max(1,ceil(rand(n,1)*length(emitlaneID)));
    laneID = emitlaneID(laneIdx);
    
    emitTable = [emitTable; [time_ vel_ laneID]];
end

[~,idx] = sort(emitTable(:,1));
emitTable = emitTable(idx,:);

end