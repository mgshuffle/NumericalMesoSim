%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway0
function vehicle = LaneChange(vehicle,CellIdx,laneCount)
	%addpath(genpath(pwd));%调用子文件夹内Dijkstra算法
	%ForcedLCIdx = find(vehicle(:,))%forcedLC

	DenMax = 1/6;%veh/meter

	[uniCIdx,~] = unique(CellIdx);
	for k = 1:length(uniCIdx)
		theIdx = find(CellIdx==uniCIdx(k));
		tmpIdx = theIdx(vehicle(theIdx,4)==max(vehicle(theIdx,4)));
		theLaneID = vehicle(theIdx(1),3);
		head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
		tail = min(vehicle(theIdx,4));
		if theLaneID==1
			vehCountL = ceil((head-tail)*DenMax);
		else
			tmpVehsL = vehicle(vehicle(:,3)==theLaneID-1);
			tmpVehsL = tmpVehsL((tmpVehsL(:,4)-head).*(tmpVehsL(:,4)-tail)<=0);
			vehCountL = length(tmpVehsL);
		end
		if theLaneID==laneCount
			vehCountR = ceil((head-tail)*DenMax);
		else
			tmpVehsR = vehicle(vehicle(:,3)==theLaneID+1);
			tmpVehsR = tmpVehsR((tmpVehsR(:,4)-head).*(tmpVehsR(:,4)-tail)<=0);
			vehCountR = length(tmpVehsR);
		end
		if vehCountL>vehCountR
			firstFlag = 0;
		else
			firstFlag = 1;
		end

		while length(theIdx)>=1
			IdxOFIdx = max(ceil(rand*length(theIdx)),1);
			LCIdx = theIdx(IdxOFIdx);
			if firstFlag==0
			else
			end
			theIdx(IdxOFIdx) = [];
		end
	end
end