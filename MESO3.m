%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
function [vehicle,D,nextLane] = MESO3(vehicle,D,nextLane,CellIdx)

global linkLen
global simStep
%global VelPass
global laneList
global leff
global linkGraph
global currentTime
global ETT

global crashTimes

%SDFpara=[50/3.6,15.6946188969802/3.6,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
%SDFpara=[50/3.6,0,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
SDFpara = [17,0,0.180,1.8,5];%default setting
CFPara=[17,91.44];%default setting

%initial
newVel = zeros(length(vehicle(:,1)),1);
newPos = zeros(size(newVel));
thisLane = vehicle(:,3);
[IY,IX] = YinX2(laneList(:,1),thisLane);
thisLane(IY) = IX;
VFree = CFPara(1);

%update vel and pos
[uniCIdx,~] = unique(CellIdx);
for i = 1:length(uniCIdx)
    theCell = find(CellIdx==uniCIdx(i));
    tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));    
    head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
    tail = min(vehicle(theCell,4));
    K = length(theCell)/(head-tail);%veh/m
    
    %speed update
    tailVel = SD(K,SDFpara);    
    %若头车接近link末尾且处于不可通行的状态时，头车速度取0
    if linkLen-veh(tmpIdx,4)<1e0 && (vehicle(tmpIdx,2)==2 || laneList(thisLane(tmpIdx),4)==0 || ETT(ETT(:,1)==vehicle(tmpIdx,3),3)>currentTime)
        headVel = 0;
    end    
    if vehicle(tmpIdx,6)~=0
        %存在前车
        leadingIdx = find(vehicle(:,1)==vehicle(tmpIdx,6));
        headVel = CF(vehicle(tmpIdx,10),vehicle(leadingIdx,5),CFPara);
    else
        if vehicle(tmpIdx,2)==2%若头车处于等待状态则vel=0
            headVel = 0;
        else
            %headVel = CF(vehicle(tmpIdx,10),VelPass,CFPara);%带通过速度
            headVel = VFree;%无前车即自由流速度通过
        end
    end
    
    %vehicle advance    
    if length(theCell)>1
        newVel(theCell) = (vehicle(theCell,4)-tail)/(vehicle(tmpIdx,4)-tail)*(headVel-tailVel)+tailVel;
    else
        newVel(theCell) = headVel;
    end    
    newVel(vehicle(:,2)==2) = 0;%waiting vehs
    
    newPos(theCell) = vehicle(theCell,4) + newVel(theCell)*simStep;
    idx_f = theCell(vehicle(theCell,6)~=0);
    CrSpace = vehicle(idx_f,4) + vehicle(idx_f,10) - leff;
    idx_idxCrash = find(newPos(idx_f)>CrSpace);
    idxCrash = idx_f(idxCrash_);
    crashTimes = crashTimes + length(idxCrash);
    newPos(idxCrash) = CrSpace(idx_idxCrash);
    
    %transpose
    %updateLane
    idx_LPass = find(newPos<linkLen & newPos>=laneList(thisLane,3)+laneList(thisLane,2));
    if ~isempty(idx_LPass)
        newLaneIdx = laneList(thisLane(idx_LPass),4);
        if ~isempty(find(newLaneIdx==0))
            %若冲入封闭车道，则引发错误
            error('冲入封闭车道');
        else
            vehicle(idx_LPass,3) = laneList(newLaneIdx,1);
            nextLane(idx_LPass) = GetNextLane(newLaneIdx,D{idx_LPass},linkGraph);
        end
    end
    
end

vehicle(:,4:5)=[newPos newVel];

%update timeBuffer
vehicle(:,9)=max(0,vehicle(:,9)-1);

arrivalIdx = find(newPos>=linkLen);
if ~isempty(arrivalIdx)
    vehicle(arrivalIdx,2)=1;
    vehicle=rmFlag(vehicle,arrivalIdx);
end
%remove arrival vehs
vehicle(arrivalIdx,:)=[];
D(arrivalIdx)=[];
nextLane(arrivalIdx)=[];

deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
if ~isempty(deadVehIdx)
    vehicle=rmFlag(vehicle,deadVehIdx);
end
%remove dead virtualVehs
vehicle(deadVehIdx,:)=[];
D(deadVehIdx) = [];
nextLane(deadVehIdx) = [];

%update space and headway
fVehsIdx = find(vehicle(:,6)~=0);%follow = leading veh is not null
lVehIdx = YinX(vehicle(fVehsIdx,6),vehicle(:,1));
vehicle(fVehsIdx,10) = vehicle(lVehIdx,4)-vehicle(fVehsIdx,4);
nonIdx = find(vehicle(:,6)==0);
vehicle(nonIdx,10) = linkLen - vehicle(nonIdx,4);
vehicle(:,11) = vehicle(:,10)./vehicle(:,5);
end

function velEql = SD(K,para)
vMax = para(1);
vMin = para(2);
kJam = para(3);
a = para(4);
b = para(5);
if K<=kJam
    velEql = vMin + (vMax - vMin)*(1-(K/kJam).^a).^b;
else
    velEql = 0;
end
end

function Vf = CF(space,Vl,para)

global simStep
global leff

VFree = para(1);
dUpper = para(2);
if space<=1e0
    Vf = Vl;
else
    if space<dUpper
        Vf = min(space/dUpper*(space-leff)/simStep + (1-space/dUpper)*Vl, 120/3.6);%120km/h：物理极限
    else
        Vf = VFree;
    end
end
end