%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
%record: #1_FID #2_VID #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway

%parameters setting
simStep = 0.1; %s
tBuff = 3; %s

%initial
load('emitTable')%format: #1_time #2_vel(m/s) #3_laneID
laneCount = 6; 
vehicle = zeros(0,11);
record = zeros(0,11);
laneRec = zeros(laneCount,2);%format: #1_lastVID #2_emitTime
vehCount = 0;

for i = 1:1000

	%发车
	while(~isempty(emitTable) && i*simStep<=emitTable(1,1))

		vehCount = vehCount + 1;

		lastVehIdx = find(vehicle(:,1)==laneRec(emitTable(1,3),1));
		if ~isempty(lastVehIdx) && vehicle(lastVehIdx,3)==emitTable(1,3)
			lastVeh = vehicle(lastVehIdx,:);
			vehicle(lastVehIdx,7) = vehCount;
			newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
					  lastVeh(1) 0 0 0 vehicle(lastVehIdx,4) i*simStep-laneRec(emitTable(1,3),2)];
		else%leading VEH GONE ALREADY
			newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
					  0 0 0 0 inf inf];
		end

		vehicle = [vehicle;newVeh];

		laneRec(emitTable(1,3),:) = [vehCount i*simStep];%update laneRec
		emitTable(1,:) = [];%remove veh emitted
	end

	if ~isempty(vehicle)
		%车团识别
		[CellIdx,indicateNum,VelState,HwState]=CellCut(vehicle,laneCount);
		%换道
		%重新识别
		%纵向运动
		
		%记录数据
		%去除到达车辆GONEVEH
	end

end