function [emitTable] = emitTableGenerater(ODList,vlim,laneCount)

emitlaneID = (1:laneCount)';

emitTable = [];
%ODIdx = ODList(:,1);
demand = ODList(:,2);
fTime = ODList(:,3);
tTime = ODList(:,4);
for i = 1:length(demand)
    time_ = rand(demand(i),1)*(tTime(i)-fTime(i)) + fTime(i);
    vel_ = rand(demand(i),1)*(vlim(2)-vlim(1)) + vlim(1);
    laneIdx = max(1,ceil(rand(demand(i),1)*length(emitlaneID)));
    laneID = emitlaneID(laneIdx);
    emitTable = [emitTable; [time_ vel_ laneID]];
end
[~,idx] = sort(emitTable(:,1));
emitTable = emitTable(idx,:);
end