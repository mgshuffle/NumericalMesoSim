function [emitTable,D] = emitTableGenerater2(ODList,vlim,laneList)

emitlaneID = laneList((laneList(:,3)==0),1);

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

vehcount = length(idx);
D = cell(vehcount,1);
rateR = 0;%右转率
rateL = 0;%左转率
RTurnIdx = randperm(vehcount,round(rateR*length(idx)));
LTurnIdx = randperm(vehcount-length(RTurnIdx),round(rateL*length(idx)));
tmp = setdiff(idx,RTurnIdx);
LTurnIdx = tmp(LTurnIdx);
flag = zeros(vehcount,1);
flag(RTurnIdx) = 1;
flag(LTurnIdx) = 2;
for k = 1:vehcount
	if flag(k)==1
		D{k} = 6;
    else
        if flag(k)==2
            D{k} = 4;
        else
            D{k} = [4 5 6];
        end
	end
end
end