%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway
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


%---------------------parameters setting----------------------------
simStep = 0.1; %s
duriation = 3600; %s
tBuff = 2; %s

linkLen = 500;%m
laneCount = 6;

ODList = [1 10000 0 3600];%1#ODID 2#demand 3#start 4#end

vlim = [5 15];%initial vel limit
VelPass = 20/3.6;%m/s
DenMax = 1/6;%veh/meter
leff = 6;%m

RCMax = 1e6;
%---------------------parameters setting----------------------------


%---------------------------initial-----------------------------------
vehicle = zeros(0,11);
record = zeros(RCMax,13);
recordCount = 0;
patchID = 1;
vehCount = 0;
frameBuff = ceil(tBuff/simStep);

%get emitTable
%load('emitTable')%format: #1_time #2_vel(m/s) #3_laneID
emitTable = emitTableGenerater(ODList,vlim,laneCount);
%---------------------------initial-----------------------------------


for i = 0:ceil(duriation/simStep)%frame loop
    %
    while(~isempty(emitTable) && i*simStep>=emitTable(1,1))
        
        vehCount = vehCount + 1;
        
        lastVehIdx = find(vehicle(:,3)==emitTable(1,3)&vehicle(:,7)==0);
        if ~isempty(lastVehIdx)
            if vehicle(lastVehIdx,4)<=leff%too close
                newVeh = [];
                vehCount = vehCount - 1;
            else
                vehicle(lastVehIdx,7) = vehCount;
                newVeh = [vehCount 0 emitTable(1,3) 0 emitTable(1,2) ...
                    vehicle(lastVehIdx,1) 0 0 0 vehicle(lastVehIdx,4) vehicle(lastVehIdx,4)/emitTable(1,2)];
            end
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
        vehicle = MESO_NoEmitTime(vehicle,CellIdx,linkLen,simStep,VelPass);%move in a frame (movement from t=i to t=i+1)
        
        %debug
        leading = vehicle(:,6);
        leading = leading(leading~=0);
        unileading = unique(leading);
        if length(unileading)~=length(leading)
            %warning('following same veh')
            a = hist(leading,unileading);
            disp('重复车');
            disp(unileading(a>1))
            error('重复车');
        end
        following = vehicle(:,7);
        following = following(following~=0);
        unifollowing = unique(following);
        if length(unifollowing)~=length(following)
            b = hist(following,unifollowing);
            disp('重复车');
            disp(unifollowing(b>1))
            error('重复车');
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
        
%         %output
%         for v = 1:length(vehicle(:,1))
%             fprintf(fileID2,fmtMat,[vehicle(v,:) (i+1)*simStep i+1]);
%         end
    
    end
    
%     %debug density
%     frVehNum = length(find(vehicle(:,8)~=0));
%     if frVehNum/linkLen/laneCount>DenMax
%         error('exceed Global maximum density')
%     end
%     closeIdx = find(vehicle(:,6)~=0&vehicle(:,10)<leff);
%     if ~isempty(closeIdx)
%         error('too close')
%     end

    disp(i*simStep)
    
end

% %output
% fclose(fileID2);

if recordCount>0
    data = record(1:recordCount,:);
    save(['record_' num2str(patchID)],'data');
end

% profsave