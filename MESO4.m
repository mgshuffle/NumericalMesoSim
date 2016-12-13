%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
function [vehicle,D,nextLane] = MESO4(vehicle,D,nextLane,CellIdx)

global linkLen
%global simStep
%global VelPass
global laneList
%global leff
global linkGraph
global currentTime
global ETT
global capacity

%global crashTimes

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
    %若头车接近segment末尾且处于不可通行的状态时，头车速度取0
    if (sum(laneList(thisLane(tmpIdx),2:3))-vehicle(tmpIdx,4)<2e0) && ...
            (vehicle(tmpIdx,2)==2 || ...
            sum(laneList(thisLane(tmpIdx),[3,5]))==0 || ...
            ( sum(laneList(thisLane(tmpIdx),[3,4]))==linkLen && ...
                ETT(ETT(:,1)==vehicle(tmpIdx,3),3)>currentTime))
        headVel = 0;
    else
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
    end    
    
    %vehicle advance
    [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel);
    
end

%Lane transpose
idx_LPass = find(newPos<linkLen & newPos>=laneList(thisLane,3)+laneList(thisLane,2));
if ~isempty(idx_LPass)
    newLaneIdx = laneList(thisLane(idx_LPass),5);
    if ~isempty(find(newLaneIdx==0))
        %若冲入封闭车道，则引发错误
        warning('即将冲入封闭车道');
        newVel(idx_LPass(newLaneIdx==0)) = 0;
        newPos(idx_LPass(newLaneIdx==0)) = vehicle(idx_LPass(newLaneIdx==0),4);
    else
        vehicle(idx_LPass,3) = laneList(newLaneIdx,1);
        for iPss = 1:length(idx_LPass)
            nextLane(idx_LPass(iPss)) = GetNextLane(newLaneIdx(iPss),D{idx_LPass(iPss)},linkGraph);
        end
    end
end

%Link transpose
for idxETT=1:length(ETT(:,1))
    arrivalIdx = find(newPos>=linkLen & vehicle(:,3)==ETT(idxETT,1));
    if ~isempty(arrivalIdx)        
        if ETT(idxETT,3)<=currentTime
            pssNum = floor((currentTime-ETT(idxETT,3))*capacity) + 1;
            if length(arrivalIdx) <= pssNum
                ETT(idxETT,3) = ETT(idxETT,3) + length(arrivalIdx)/capacity;
                vehicle(arrivalIdx,2)=1;
            else
                [~,ord_pss] = sort(vehicle(arrivalIdx,4));
                arrivalIdx = arrivalIdx(ord_pss);
                nopssIdx = arrivalIdx(pssNum+1:end);
                arrivalIdx = arrivalIdx(1:pssNum);
                ETT(idxETT,3) = ETT(idxETT,3) + length(arrivalIdx)/capacity;
                CIDs = unique(CellIdx(nopssIdx));
                for ci = 1:length(CIDs)
                    theCell = find(CellIdx==CIDs(ci));
                     tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));    
                     head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
                     tail = min(vehicle(theCell,4));
                     K = length(theCell)/(head-tail);%veh/m
                     %speed update
                     tailVel = SD(K,SDFpara);
                     headVel = 0;
                    [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel);
                end
            end
            vehicle=rmFlag(vehicle,arrivalIdx);
            %remove arrival vehs
            vehicle(arrivalIdx,:)=[];
            D(arrivalIdx)=[];
            nextLane(arrivalIdx)=[];
            newPos(arrivalIdx)=[];
            newVel(arrivalIdx)=[];
            CellIdx(arrivalIdx)=[];
        else
            pssNum = 0;
            nopssIdx = arrivalIdx(pssNum+1:end);
            arrivalIdx = arrivalIdx(1:pssNum);
            ETT(idxETT,3) = ETT(idxETT,3) + length(arrivalIdx)/capacity;
            CIDs = unique(CellIdx(nopssIdx));
            for ci = 1:length(CIDs)
                theCell = find(CellIdx==CIDs(ci));
                tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));
                head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
                tail = min(vehicle(theCell,4));
                K = length(theCell)/(head-tail);%veh/m
                %speed update
                tailVel = SD(K,SDFpara);
                headVel = 0;
                [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel);
            end
        end
    end
end


% arrivalIdx = find(newPos>=linkLen);
% if ~isempty(arrivalIdx)
%     idxETT = find(ETT(:,1)==vehicle(arrivalIdx(1),3));
%     if ETT(idxETT,3)<=currentTime
%         pssNum = floor((currentTime-ETT(idxETT,3))*capacity) + 1;
%         if length(arrivalIdx) <= pssNum
%             ETT(idxETT,3) = ETT(idxETT,3) + length(arrivalIdx)/capacity;
%             vehicle(arrivalIdx,2)=1;
%         else
%             [~,ord_pss] = sort(vehicle(arrivalIdx,4));
%             arrivalIdx = arrivalIdx(ord_pss);
%             tmpIdx = arrivalIdx(pssNum+1);
%             arrivalIdx = arrivalIdx(1:pssNum);
%             ETT(idxETT,3) = ETT(idxETT,3) + length(arrivalIdx)/capacity;
%             theCell = setdiff(theCell,arrivalIdx);
%             headVel = 0;
%             [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel);
%         end
%         vehicle=rmFlag(vehicle,arrivalIdx);
%         %remove arrival vehs
%         vehicle(arrivalIdx,:)=[];
%         D(arrivalIdx)=[];
%         nextLane(arrivalIdx)=[];
%         newPos(arrivalIdx)=[];
%         newVel(arrivalIdx)=[];
%     else
%         %pssNum = 0;
%         headVel = 0;
%         [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel);
%     end    
% end

vehicle(:,4:5)=[newPos newVel];

%update timeBuffer
vehicle(:,9)=max(0,vehicle(:,9)-1);

%remove dead virtualVehs
deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
if ~isempty(deadVehIdx)
    vehicle=rmFlag(vehicle,deadVehIdx);
end
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

%debug
idx_over = find(vehicle(:,4)>=linkLen);
if ~isempty(idx_over)
    error('link transposed vehs not removed')
end
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

function [vehicle,newVel,newPos] = updateSpeed (vehicle,newVel,newPos,theCell,tmpIdx,tail,tailVel,headVel)

global simStep
global leff
global crashTimes

if length(theCell)>1
    newVel(theCell) = (vehicle(theCell,4)-tail)/(vehicle(tmpIdx,4)-tail)*(headVel-tailVel)+tailVel;
else
    newVel(theCell) = headVel;
end
newVel(vehicle(:,2)==2) = 0;%waiting vehs

newPos(theCell) = vehicle(theCell,4) + newVel(theCell)*simStep;
idx_f = theCell(vehicle(theCell,6)~=0);
if ~isempty(idx_f)
    CrSpace = vehicle(idx_f,4) + vehicle(idx_f,10) - leff;
    idx_idxCrash = find(newPos(idx_f)>CrSpace);
    if ~isempty(idx_idxCrash)
        idxCrash = idx_f(idx_idxCrash);
        crashTimes = crashTimes + length(idxCrash);
        newPos(idxCrash) = CrSpace(idx_idxCrash);
        newVel(idxCrash) = (CrSpace(idx_idxCrash) - vehicle(idxCrash,4))/simStep;
    end
end
end