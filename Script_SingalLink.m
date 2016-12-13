%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual>0) #9_(adjust_time_)remain(>=0) #10_space #11_headway
%record: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading
%#7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space
%#11_headway #12_Time #13_FID

% %-----------------------------output setting-----------------------------
% outfile = './record.csv';
% fileID = fopen(outfile, 'w');
% %if fid == -1; error('Cannot open file: %s', outfile); end
% header = {'VID','State','laneID','S','Vel','leading','following','type','remain','space','headway','Time','FID'};
% fmtString = [repmat('%s,',1,length(header)-1) '%s\r\n'];
% fprintf(fileID, fmtString, header{:});
% fclose(fileID);
% fileID2 = fopen(outfile,'a');
% if fileID2 == -1; 
%     error('Cannot open file: %s', outfile); 
% end
% fmtMat = [repmat('%G,',1,length(header)-1) '%G\r\n'];
% %-----------------------------output setting-----------------------------

global simStep
global duriation
global tBuff
global linkLen
global ODList
global vlim
global VelPass
global leff
global DenMax
global capacity %perlane persec

global frameBuff
global laneList
global linkGraph
global currentTime
global ETT

global crashTimes
%---------------------parameters setting----------------------------
simStep = 0.1; %s
ETStep = 10;
LCStep = 1;
duriation = 3600; %s
tBuff = 2; %s

linkLen = 600;%m

ODList = [1 10000 0 3600];%1#ODID 2#demand 3#start 4#end

vlim = [5 15];%initial vel limit
VelPass = 20/3.6;%m/s
capacity = 0.5;
leff = 6;%m
DenMax = 1/leff;%veh/meter

RCMax = 1e6;
%---------------------parameters setting----------------------------


%---------------------------initial-----------------------------------
vehicle = zeros(0,11);
D = cell(0,1);
nextLane = zeros(0,1);
record = zeros(RCMax,13);
recordCount = 0;
patchID = 1;
vehCount = 0;
frameBuff = ceil(tBuff/simStep);

ETClock = ETStep;
LCClock = LCStep;
emitVCount = 0;

crashTimes = 0;

%get network
[laneList,linkGraph,ETT] = NetworkGenerater();
%get emitTable
%load('emitTable')%format: #1_time #2_vel(m/s) #3_laneID
[emitTable,emitD] = emitTableGenerater2(ODList,vlim,laneList);
%---------------------------initial-----------------------------------


for i = 0:ceil(duriation/simStep)%frame loop
    currentTime = i*simStep;
    
%     %debug
%     if mod(currentTime,20)==0
%     end
    %
    while(~isempty(emitTable) && i*simStep>=emitTable(1,1))
        
        vehCount = vehCount + 1;
        thisLane = vehicle(:,3);
        [IY,IX] = YinX2(laneList(:,1),thisLane);
        thisLane(IY)=IX;
        lastVehIdx = find(laneList(thisLane,4)==laneList(laneList(:,1)==emitTable(1,3),4)&vehicle(:,7)==0);
        if ~isempty(lastVehIdx)
            if vehicle(lastVehIdx,4)<=leff%too close
                newVeh = [];
                vehCount = vehCount - 1;
            else
                emitVCount = emitVCount + 1;
                vehicle(lastVehIdx,7) = vehCount;
                newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
                    vehicle(lastVehIdx,1) 0 0 0 vehicle(lastVehIdx,4) vehicle(lastVehIdx,4)/emitTable(1,2)];
            end
        else%leading VEH GONE ALREADY
            newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
                0 0 0 0 linkLen linkLen/emitTable(1,2)];
        end
        
        if ~isempty(newVeh)
            vehicle = [vehicle;newVeh];
            D{length(D)+1} = emitD{1};
            nextLane = [nextLane; GetNextLane(find(laneList(:,1)==newVeh(3)),D{length(D)},linkGraph)];
        end
        
        emitTable(1,:) = [];%remove veh emitted
        emitD(1) = [];
    end
    
    %reset emitTime
    if currentTime>=ETClock
        ETT(:,3) = currentTime;
        ETClock = ETClock + ETStep;
    end
    
    if ~isempty(vehicle)
        %����ʶ��
        [CellIdx,~,~,~]=CellCut(vehicle);
        if currentTime>=LCClock
            LCClock = LCClock + LCStep;
            %��������
            [vehicle,D,nextLane,vehCount] = LaneChange4(vehicle,D,nextLane,vehCount,CellIdx);
            %debug
            IDX = crashTest(vehicle);
            if ~isempty(IDX)
            end
            %������ʶ��
            [CellIdx,~,~,~]=CellCut(vehicle);
        end
        %move in a frame (movement from t=i to t=i+1)
        [vehicle,D,nextLane] = MESO4(vehicle,D,nextLane,CellIdx);
        %debug
        IDX = crashTest(vehicle);
        if ~isempty(IDX)
        end

        %debug
        leading = vehicle(:,6);
        leading = leading(leading~=0);
        unileading = unique(leading);
        if length(unileading)~=length(leading)
            %warning('following same veh')
            a = hist(leading,unileading);
            disp(unileading(a>1))
        end
        following = vehicle(:,7);
        following = following(following~=0);
        unifollowing = unique(following);
        if length(unifollowing)~=length(following)
            b = hist(following,unifollowing);
            disp(unifollowing(b>1))
        end
        
        %add record
        num2add = length(vehicle(:,1));
        if recordCount + num2add > RCMax%too many record, then -->output-->release
            %output
            data = record(1:recordCount,:);
            save(['record_' num2str(patchID)],'data');
            %recycle
            record = record * 0;
            recordCount = 0;
            patchID = patchID + 1;
        end
        %vehs in one frame must not more than RCMax
        record(recordCount+1:recordCount+num2add,:) = [vehicle (i+1)*simStep*ones(num2add,1) (i+1)*ones(num2add,1)];
        recordCount = recordCount + num2add;
    
    end

    disp(i*simStep)
    
end

if recordCount>0
    data = record(1:recordCount,:);
    save(['record_' num2str(patchID)],'data');
end

% profsave