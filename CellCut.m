%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
function [CellNum,indicateNum,VelState,HwState]=CellCut(vehicle)
[VelState, HwState] = StateDetect(vehicle);
indicateNum = zeros(length(vehicle(:,1)),1);

fVehsIdx = find(vehicle(:,6)~=0);%follow = leading veh is not null
if ~isempty(fVehsIdx)
    lVehIdx = YinX(vehicle(fVehsIdx,6),vehicle(:,1));
    if length(lVehIdx)~=length(fVehsIdx)
        error('vehs do not match!');
    else
        indicateNum(fVehsIdx) = VelState(fVehsIdx)==VelState(lVehIdx);%vehs speedState same as leadingVeh
    end
    indicateNum = indicateNum&HwState;%vehs speedState same as leadingVeh and also in following headway
end

CellNum = 1:length(vehicle(:,1));
flag = indicateNum;

idx = find(flag==1);
while ~isempty(idx)
    theLVehIdx = YinX(vehicle(idx,6),vehicle(:,1));
    if length(theLVehIdx)~=length(theLVehIdx)
        error('vehs do not match');
    end
    CellNum(idx)=CellNum(theLVehIdx);
    flag(idx(flag(theLVehIdx)==0)) = 0;
    idx = find(flag==1);
end
end

function [VelState, HwState]=StateDetect(vehicle)
VelState = floor(vehicle(:,5)/2) + 1;
HwState = ~(vehicle(:,11)>5);
end