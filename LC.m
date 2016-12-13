function [vehicle,D,nextLane,theLPosVC,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,theLPosVC,totVehCount,LPVehIdx,LPos,LCIdx,theTurn)
global laneList
global linkGraph
global frameBuff
global linkLen

% %debug
% if theTurn==1 && vehicle(LCIdx,3)==105
% end

laneIDList = laneList(:,1);
LCVID = vehicle(LCIdx,1);
theLaneIdx = find(laneIDList==vehicle(LCIdx,3));
theLPIdx = find(LPos == laneList(theLaneIdx,4));
theTPIdx = find(LPos==laneList(theTurn,4));

%update countNum
theLPosVC = theLPosVC + 1;
totVehCount = totVehCount + 1;

%update poperties in vehList: turn to virtual Veh
vehicle(LCIdx,[1 8 9]) = [totVehCount LCVID frameBuff];
vehicle(vehicle(:,6)==LCVID,6) = totVehCount;
vehicle(vehicle(:,7)==LCVID,7) = totVehCount;

%update poperties of newPosVeh
newPosVeh = vehicle(LCIdx,:);
newPosVeh([1 3 8 9]) = [LCVID laneIDList(theTurn) 0 frameBuff];
idx_tgs = LPVehIdx{theTPIdx};

idx_tgs_front = idx_tgs(vehicle(idx_tgs,4)>vehicle(LCIdx,4));
idx_tgs_rear = idx_tgs(vehicle(idx_tgs,4)<=vehicle(LCIdx,4));

if isempty(idx_tgs_front)
    newPosVeh(6)=0;
    newPosVeh(10) = linkLen - newPosVeh(4);
else
    [~,tmp] = min(vehicle(idx_tgs_front,4));
    idx_front = idx_tgs_front(tmp);
    newPosVeh(6) = vehicle(idx_front,1);
    vehicle(idx_front,7) = newPosVeh(1);
    newPosVeh(10) = vehicle(idx_front,4) - newPosVeh(4);
end
newPosVeh(11) = newPosVeh(10)/newPosVeh(5);

if isempty(idx_tgs_rear)
    newPosVeh(7)=0;
else
    [~,tmp] = max(vehicle(idx_tgs_rear,4));
    idx_rear = idx_tgs_rear(tmp);
    newPosVeh(7) = vehicle(idx_rear,1);
    vehicle(idx_rear,6) = newPosVeh(1);
    vehicle(idx_rear,10) = newPosVeh(4)-vehicle(idx_rear,4);
    vehicle(idx_rear,11) = vehicle(idx_rear,10) / vehicle(idx_rear,5);
end

%update laneVIdx
LPVehIdx{theTPIdx} = [LPVehIdx{theTPIdx}; length(vehicle(:,1))+1];

vehicle = [vehicle;newPosVeh];
D{length(D)+1} = D{LCIdx};
nextLane(length(nextLane)+1,1) = GetNextLane(theTurn,D{end},linkGraph);

end