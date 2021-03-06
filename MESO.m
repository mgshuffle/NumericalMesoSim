%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
function vehicle = MESO(vehicle,CellIdx,linkLen,simStep,VelPass)

global emitTime
global currentTime
global capacity

%SDFpara=[50/3.6,15.6946188969802/3.6,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
%SDFpara=[50/3.6,0,114.313208506104/1000,2.21680315505852,9.98014217655979];%i80-2
SDFpara = [17,0,0.180,1.8,5];
CFPara=[17,91.44];
leff = 6;

%initial
newVel = zeros(length(vehicle(:,1)),1);
newPos = zeros(size(newVel));

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
        %headVel = CF(vehicle(tmpIdx,10),VelPass,CFPara,simStep,leff);
        headVel = CFPara(1);
        if vehicle(tmpIdx,10)<=headVel*simStep
            if currentTime>=emitTime
                emitTime = emitTime + 1/capacity;
            else
                headVel = 0;
            end
        end
    end
    %[~,order_]sort(vehicle(theCell),4);
    %theCell = theCell(order_);
    if length(theCell)>1
        newVel(theCell) = (vehicle(theCell,4)-tail)/(vehicle(tmpIdx,4)-tail)*(headVel-tailVel)+tailVel;
    else
        newVel(theCell) = headVel;
    end
    newPos(theCell) = vehicle(theCell,4) + newVel(theCell)*simStep;
    idx_f = theCell(vehicle(theCell,6)~=0);
    if ~isempty(idx_f)
        newPos(idx_f) = min(newPos(idx_f), vehicle(idx_f,4) + vehicle(idx_f,10) - 1e-1);
    end
end
vehicle(:,4:5)=[newPos newVel];

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
vehicle(vehicle(:,2)==1,:)=[];

deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
if ~isempty(deadVehIdx)
    vehicle=rmFlag(vehicle,deadVehIdx);
end
%remove dead virtualVehs
vehicle(vehicle(:,8)&~vehicle(:,9),:)=[];

% arrivalIdx = find(newPos>=linkLen);
% vehicle(arrivalIdx,2)=1;
% deadVehIdx = find(vehicle(:,8)&~vehicle(:,9));
% goneIdx = union(arrivalIdx,deadVehIdx);
% if ~isempty(goneIdx)
%     vehicle = rmFlag(vehicle,deadVehIdx);
% end





%update space and headway
fVehsIdx = find(vehicle(:,6)~=0);%follow = leading veh is not null
%debug
wrongVehs = setdiff(vehicle(fVehsIdx,6),vehicle(:,1));
if ~isempty(wrongVehs)
    disp(vehicle(YinX(wrongVehs,vehicle(:,6)),:));
end
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