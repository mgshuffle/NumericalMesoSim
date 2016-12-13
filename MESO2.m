%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
function [vehicle,D,nextLane] = MESO2(vehicle,D,CellIdx,linkLen,simStep,VelPass,laneList,nextLane,linkGraph)

%SDFpara=[50/3.6,15.6946188969802/3.6,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
SDFpara=[50/3.6,0,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
CFPara=[50/3.6,180];
leff = 6;

%initial
newVel = zeros(length(vehicle(:,1)),1);
newPos = zeros(size(newVel));
thisLane = vehicle(:,3);
[IY,IX] = YinX2(laneList(:,1),thisLane);
thisLane(IY) = IX;

%update vel and pos
[uniCIdx,~] = unique(CellIdx);
for i = 1:length(uniCIdx)
    theCell = find(CellIdx==uniCIdx(i));
    tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));
    head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
    tail = min(vehicle(theCell,4));
    K = length(theCell)/(head-tail);%veh/m
    tailVel = SD(K,SDFpara);
    if vehicle(tmpIdx,6)~=0
        leadingIdx = find(vehicle(:,1)==vehicle(tmpIdx,6));
        headVel = CF(vehicle(tmpIdx,10),vehicle(leadingIdx,5),CFPara,simStep,leff);
    else
        %headVel = CF(vehicle(tmpIdx,10),VelPass,CFPara);
        headVel = CFPara(1);
    end
    %[~,order_]sort(vehicle(theCell),4);
    %theCell = theCell(order_);
    if length(theCell)>1
        newVel(theCell) = (vehicle(theCell,4)-tail)/(vehicle(tmpIdx,4)-tail)*(headVel-tailVel)+tailVel;
    else
        newVel(theCell) = headVel;
    end
    
    idx_f = theCell(vehicle(theCell,6)~=0);
    if ~isempty(idx_f)
        newVel(idx_f) = min(newVel(idx_f), (vehicle(idx_f,4) + vehicle(idx_f,10) - leff)/simStep);%veh too fast
    end
    newVel(vehicle(:,2)==2) = 0;%waiting vehs
    
    newPos(theCell) = vehicle(theCell,4) + newVel(theCell)*simStep;
    if ~isempty(idx_f)
        newVel(idx_f) = min(newVel(idx_f), (vehicle(idx_f,4) + vehicle(idx_f,10) - 0.1)/simStep);%veh too fast
    end
    
    %ensure no vehs enter blocked lanes
    prestopVIdx = find(newPos>=laneList(thisLane,3)+laneList(thisLane,2) & newPos<linkLen);
    if ~isempty(prestopVIdx)
        tmpval = linkGraph(thisLane(prestopVIdx)+length(linkGraph(:,1))*(nextLane(prestopVIdx)-1));
%         stopVIdx = prestopVIdx(tmpval~=1);
%         if ~isempty(stopVIdx)
%             newVel(stopVIdx) = 0;%blocked lanes
%             if ~isempty(idx_f)
%                 newVel(idx_f) = min(newVel(idx_f), (vehicle(idx_f,4) + vehicle(idx_f,10) - leff)/simStep);%veh too fast
%             end
%             newPos(theCell) = vehicle(theCell,4) + newVel(theCell)*simStep;
%         end
        %pass this lane
        passVIdx = prestopVIdx(tmpval==1);
        if ~isempty(passVIdx)
            if ~isempty(find(vehicle(passVIdx,3)==5)) || ~isempty(find(vehicle(passVIdx,3)==6))
            end
            vehicle(passVIdx,3) = laneList(nextLane(passVIdx),1);
            for II = 1:length(passVIdx)
                psv = passVIdx(II);
                nextLane(psv) = GetNextLane(nextLane(psv),D{psv},linkGraph);
            end
            
        end
    end
    
end
%debug
oVehs = vehicle;
vehicle(:,4:5)=[newPos newVel];
IDX = crashTest(vehicle);
if ~isempty(IDX)
end

%update timeBuffer
vehicle(:,9)=max(0,vehicle(:,9)-1);

%flag the arrived veh and mark corresponding followingVeh as nonLeadingVeh
%vehicle(newPos>=linkLen,2)=1;
%renewFVID = vehicle(newPos>=linkLen,7);
%renewFVID = renewFVID(renewFVID~=0);
%vehicle(YinX(renewFVID,vehicle(:,1)),6)=0;

%find the dead virtualVehs and mark corresponding followingVehs and leadingVehs
%deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
%LVID = vehicle(deadVehIdx,6);
%LVID = LVID(LVID~=0);
%subLVehIdx = deadVehIdx(LVID~=0);
%vehicle(YinX(LVID,vehicle(:,1)),7)=vehicle(subLVehIdx,7);
%FVID = vehicle(deadVehIdx,7);
%FVID = FVID(FVID~=0);
%subFVehIdx = deadVehIdx(FVID~=0);
%vehicle(YinX(FVID,vehicle(:,1)),6)=vehicle(subFVehIdx,6);


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
% arrivalIdx = find(newPos>=linkLen);

% vehicle(arrivalIdx,2)=1;
% deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
% goneIdx = union(arrivalIdx,deadVehIdx);
% if ~isempty(goneIdx)
%     vehicle = rmFlag(vehicle,deadVehIdx);
% end

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

% function Vf = CF(space,Vl,para)
% VFree = para(1);
% dUpper = para(2);
% Vf=min(max((VFree-Vl)/dUpper*space+Vl,Vl),VFree);
% end

function Vf = CF(space,Vl,para,simStep,leff)
VFree = para(1);
dUpper = para(2);
if space<=0.1
    Vf = Vl;
else
    if space<dUpper
        Vf = min(space/dUpper*(space-leff)/simStep + (1-space/dUpper)*Vl, 120/3.6);
    else
        Vf = VFree;
    end
end
end