%vehicle: #1_VID #2_State(arrived==1,waiting==2) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual>0) #9_(adjust_time_)remain(>=0) #10_space #11_headway0
%laneList: #1_ID #2_laneLen #3_laneStart 
function [vehicle,D,nextLane,totVehCount] = LaneChange4(vehicle,D,nextLane,totVehCount,CellIdx)
%addpath(genpath(pwd));%

global laneList
global linkGraph
global linkLen
global DenMax
global leff

laneIDList = laneList(:,1);
% laneLen = laneList(:,2);
LPos = unique(laneList(:,4));
LPosCount = length(LPos);

%将LaneID转换成LanePosition的索引
vehLPList = vehicle(:,3);
[IY,IX] = YinX2(laneIDList,vehLPList);
vehLPList(IY) = laneList(IX,4);


%根据LanePos索引进行归类
for LPIdx = 1:LPosCount
    LPVehIdx{LPIdx} = find(vehLPList==LPos(LPIdx));
end
%各车道的转向车道
turning = CalTuringLane(linkGraph);

[uniCIdx,~] = unique(CellIdx);
for k = 1:length(uniCIdx)
    %取出属于该cell的veh的索引theCell，以及其中可变道车辆的veh的索引LCVehs
    theCell = find(CellIdx==uniCIdx(k));
    %LCVehs = theCell(vehicle(theCell,8)==0 & vehicle(theCell,9)==0);%
    LCVehs = theCell(vehicle(theCell,8)==0);%
    %该车道的ID,在LaneList中的索引，以及转向车道的索引
    
    %若存在可变道车辆
    if ~isempty(LCVehs) %LCV>1
        thisVehCount = length(theCell);%车队车辆数
        tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));%头车索引：位置最前的车        
        head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);%车队下游边界
        tail = min(vehicle(theCell,4));%车队上游边界
        

        CountMax = ceil((head-tail)*DenMax);
        if CountMax<1
            error('too short');
        end
        
        while length(LCVehs)>=1            
            IdxOFIdx = max(ceil(rand*length(LCVehs)),1);
            LCIdx = LCVehs(IdxOFIdx);
            theTurning = turning(laneIDList==vehicle(LCIdx,3),:);
            thelane = find(laneIDList==vehicle(LCIdx,3));
            theLP = laneList(thelane,4);
            %enLanes本segment可通行车道
            enLanes = find(laneList(:,3)==laneList(thelane,3));
            if sum(laneList(thelane,2:3))<linkLen
                [idxtmp,~] = find(linkGraph(enLanes,:)==1);
                enLanes = enLanes(idxtmp);
                if isempty(enLanes), error('拓扑错误'); end
            else
                enLanes = D{LCIdx};
            end
            if (theTurning(1)||theTurning(2)) && vehicle(LCIdx,9)==0%存在可换车道且处于可换道状态
                twoTL = theTurning(1)&theTurning(2);%flag 是否存在第二转向车道
                %换道信息
                TLane(1) = theTurning(1);
                TLPos(1) = laneList(theTurning(1),4);
                P(1) = CalPLC(vehicle(LCIdx,4),thelane,theLP,vehicle(LPVehIdx{TLPos(1)},:),head,tail,thisVehCount,TLPos(1),enLanes);
                if twoTL
                    TLane(2) = theTurning(2);
                    TLPos(2) = laneList(theTurning(2),4);
                    P(2) = CalPLC(vehicle(LCIdx,4),thelane,theLP,vehicle(LPVehIdx{TLPos(2)},:),head,tail,thisVehCount,TLPos(2),enLanes);
                    %排序
                    if P(2)>P(1)
                        TLane = TLane(2:-1:1);
                        TLPos = TLPos(2:-1:1);
                        P = P(2:-1:1);
                    end
                end
                %第一换道方向尝试
                if (rand<P(1) || vehicle(LCIdx,2)==2) && LeffTest(vehicle(LPVehIdx{TLPos(1)},:), vehicle(LCIdx,:), leff)
                    %换到第一车道
                    if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                    [vehicle,D,nextLane,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,totVehCount,LPVehIdx,LPos,LCIdx,TLane(1));
                else
                    if twoTL && (rand<P(2) || vehicle(LCIdx,2)==2) && LeffTest(vehicle(LPVehIdx{TLPos(2)},:), vehicle(LCIdx,:), leff)
                        %换到第二车道
                        if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                        [vehicle,D,nextLane,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,totVehCount,LPVehIdx,LPos,LCIdx,TLane(2));
                    else
                        %if di~=0 && PM>=eps, vehicle(LCIdx,2)=2;end
                        di = dijkstra1m(linkGraph,thelane,enLanes);
                        PM = MLC(vehicle(LCIdx,4),thelane);
                        if di~=0 && PM>=0.95
                            vehicle(LCIdx,2)=2;
                        end
                    end
                end
            else
                di = dijkstra1m(linkGraph,thelane,enLanes);
                PM = MLC(vehicle(LCIdx,4),thelane);
                if di~=0 && PM>=0.8
                    vehicle(LCIdx,2)=2;
                end
            end
            %remove LCIdx 标记位已经考虑
            LCVehs(IdxOFIdx) = [];
        end
    end
end
end

function result = LeffTest(vehTgs, vehLC, leff)
if isempty(vehTgs)
    result = true;
else
    idx_tgs_front = find(vehTgs(:,4)>vehLC(4));
    idx_tgs_rear = find(vehTgs(:,4)<=vehLC(4));
    
    if ~isempty(idx_tgs_front)
        [~,idx_front] = min(vehTgs(idx_tgs_front,4));
        a = vehTgs(idx_tgs_front(idx_front),4)-vehLC(4)>leff;
    else
        a = true;
   end
    
    if ~isempty(idx_tgs_rear)
        [~,idx_rear] = max(vehTgs(idx_tgs_rear,4));
        b = vehLC(4)-vehTgs(idx_tgs_rear(idx_rear),4)>leff;
    else
        b = true;
    end
    
    result = a&b;
end
end

function p = MLC(s,thisLIdx)
global laneList
len = sum(laneList(thisLIdx,2:3));
    buff = 20;%m
    if len > buff
        p = (min(s,len-buff))/(len-buff);
    else
        p = 1;
    end
end

function p = DLC(JLvehs,CellVC,head,tail,theTLP)
global laneList
global leff
lanes = laneList(laneList(:,4)==theTLP,:);
[m,~] = size(lanes);
a = max([lanes(:,3)';tail*ones(1,m)]);
b = min([(lanes(:,2)+lanes(:,3))';head*ones(1,m)]);
c = b-a;
s = sum(c(c>0));
tmp = JLvehs(:,4)-tail>=0 & tail-JLvehs(:,4)>0;
tmp = tmp(tmp>0);
KJL = (length(tmp) + (head-tail-s)/leff + 1)/(head-tail);
K = CellVC/(head-tail);
p = min(1,max(0,(K-KJL)/(1/leff-KJL)));
end

function p=CalPLC(vehLongPos,vehLIdx,vehLP,JLvehs,head,tail,CellVC,theTLP,enLanes)
global linkGraph
di=dijkstra1m(linkGraph,vehLP,enLanes);
dj=dijkstra1m(linkGraph,theTLP,enLanes);
d = dj - di;
if d<0
    H = 1;
else
    if d>0
        H = -1;
    else
        H = 0;
    end
end
p = H*MLC(vehLongPos,vehLIdx)+ DLC(JLvehs,CellVC,head,tail,theTLP);
p = min(1,max(0,p));
end

function [vehicle,D,nextLane,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,totVehCount,LPVehIdx,LPos,LCIdx,theTurn)
global laneList
global linkGraph
global frameBuff
global linkLen

% %debug
% if theTurn==1 && vehicle(LCIdx,3)==105
% end

laneIDList = laneList(:,1);
LCVID = vehicle(LCIdx,1);
theTPIdx = find(LPos==laneList(theTurn,4));

%update countNum
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
