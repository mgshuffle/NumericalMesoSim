%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
%record: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway #12_FID

%parameters setting
simStep = 0.1; %s
tBuff = 2; %s
linkLen = 500;%m
VelPass = 20/3.6;%m/s

%initial
load('emitTable')%format: #1_time #2_vel(m/s) #3_laneID
laneCount = 6; 
vehicle = zeros(0,11);
record = zeros(0,12);
vehCount = 0;
frameBuff = ceil(tBuff/simStep);

for i = 0:1000%frame loop
	%
	while(~isempty(emitTable) && i*simStep>=emitTable(1,1))

		vehCount = vehCount + 1;

        lastVehIdx = find(vehicle(:,3)==emitTable(1,3)&vehicle(:,7)==0);
		if ~isempty(lastVehIdx)
			vehicle(lastVehIdx,7) = vehCount;
			newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
					  vehicle(lastVehIdx,1) 0 0 0 vehicle(lastVehIdx,4) vehicle(lastVehIdx,4)/emitTable(1,2)];
		else%leading VEH GONE ALREADY
			newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
					  0 0 0 0 linkLen linkLen/emitTable(1,2)];
		end

		vehicle = [vehicle;newVeh];

		emitTable(1,:) = [];%remove veh emitted
	end

	if ~isempty(vehicle)
		%
		[CellIdx,~,~,~]=CellCut(vehicle);
		%
		[vehicle,vehCount] = LaneChange(vehicle,CellIdx,laneCount,frameBuff,vehCount,linkLen);
		%
		[CellIdx,~,~,~]=CellCut(vehicle);
		%
		vehicle = MESO(vehicle,CellIdx,linkLen,simStep,VelPass);%move in a frame (movement from t=i to t=i+1)
		
        %debug
        leading = vehicle(:,6);
        leading = leading(leading~=0);
        unileading = unique(leading);
        if length(unileading)~=length(leading)
            %warning('following same veh')
            a = hist(leading,unileading);
            disp('重复车');
            disp(unileading(a>1))
            disp('冲突车');
            disp(vehicle(YinX(unileading(a>1),vehicle(:,6)),[1 8 9]))            
        end
        following = vehicle(:,7);
        following = following(following~=0);
        unifollowing = unique(following);
        if length(unifollowing)~=length(following)
            b = hist(following,unifollowing);
            disp('重复车');
            disp(unifollowing(b>1))
            disp('冲突车')
            disp(vehicle(YinX(unifollowing(b>1),vehicle(:,7)),[1 8 9]))
        end
        
		%
		record = [record;[vehicle (i+1)*simStep*ones(length(vehicle(:,1)),1)]];
	end

end