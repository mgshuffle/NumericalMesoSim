%vehicle: #1_VID #2_State(arrived==1,waiting==2) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway0
%laneList: #1_ID #2_laneLen #3_laneStart 
function [vehicle,D,nextLane,totVehCount] = LaneChange2(vehicle,CellIdx,frameBuff,totVehCount,linkLen,nextLane,laneList,linkGraph,D)
%addpath(genpath(pwd));%
%ForcedLCIdx = find(vehicle(:,))%forcedLC

DenMax = 1/6;%veh/meter
leff = 6;%m

laneIDList = laneList(:,1);
laneLen = laneList(:,2);
laneCount = length(laneList(:,1));
%laneVeh = cell(laneCount,1);
for laneIdx = 1:laneCount
    laneVIdx{laneIdx} = find(vehicle(:,3)==laneIDList(laneIdx));
end
turning = CalTuringLane(linkGraph);

[uniCIdx,~] = unique(CellIdx);
for k = 1:length(uniCIdx)
    theCell = find(CellIdx==uniCIdx(k));
    LCVehs = theCell(vehicle(theCell,8)~=1 & vehicle(theCell,9)==0);%
    %LCVehs = LCVehs(vehicle(theCell,9)==0);%can be combined
    theLaneID = vehicle(theCell(1),3);
    theLaneIdx = find(laneIDList==theLaneID);    
    theTurning = turning(theLaneIdx,:);
    if ~isempty(LCVehs) && sum(theTurning)>0 %LCV>1 and turnable lane count>0
        thisVehCount = length(theCell);
        tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));        
        head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
        tail = min(vehicle(theCell,4));
        twoTL = theTurning(1)&theTurning(2);%flag

        CountMax = ceil((head-tail)*DenMax);
        if CountMax<1
            error('too short');
        end

        firstGroup = find(laneList(:,4)==laneList(theTurning(1),4));
        if length(firstGroup)>1
        end
        idx_FTurn = cell2mat(laneVIdx(firstGroup));

        idx_FTurn = idx_FTurn((vehicle(idx_FTurn,4)-head).*(vehicle(idx_FTurn,4)-tail)<=0);
        vehCount1 = length(idx_FTurn);
        Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );

        if twoTL
            secondGroup = find(laneList(:,4)==laneList(theTurning(2),4));
            idx_STurn = cell2mat(laneVIdx(secondGroup));
%             %debug
%             if ~isempty(idx_STurn) && (min(idx_STurn)<=0 || max(idx_STurn)>length(vehicle(:,1)))
%             end
            idx_STurn = idx_STurn((vehicle(idx_STurn,4)-head).*(vehicle(idx_STurn,4)-tail)<=0);
            vehCount2 = length(idx_STurn);
            Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );

            if vehCount2<vehCount1%order
                tmpTurn = firstGroup;
                firstTurn = secondGroup;
                secondTurn = tmpTurn;

                idx_tmpT = idx_FTurn;
                idx_FTurn = idx_STurn;
                idx_STurn = idx_tmpT;

                vehCount1 = length(idx_FTurn);
                vehCount2 = length(idx_STurn);
                Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );
                Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );
            end
        end

        while length(LCVehs)>=1
            IdxOFIdx = max(ceil(rand*length(LCVehs)),1);
            LCIdx = LCVehs(IdxOFIdx);
            %LCVID = vehicle(LCIdx,1);

            if linkGraph(laneIDList==vehicle(LCIdx,3),nextLane(LCIdx))==2%MLC
                theTurn = nextLane(LCIdx);
                PMLC = MLC(vehicle(LCIdx,4),laneLen(theTurn));
                if theTurn==firstTurn
                    P = min(1,PMLC + Pr);
                    vehTgs = vehicle(laneVIdx{firstGroup},:);
                    tmpFlag = LeffTest(vehTgs, vehicle(LCIdx,:), leff);
                    if (rand<P && tmpFlag)%Lane Changed
                        if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                        [vehCount1,totVehCount,laneVIdx,vehicle,D,nextLane] = LC(vehCount1,totVehCount,laneVIdx,vehicle,D,nextLane,LCIdx,theLaneIdx,theTurn,laneIDList,frameBuff,linkGraph,linkLen);
                        Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );
                    end
                else
                    P = min(1,PMLC + Pr2);
                    vehTgs = vehicle(laneVIdx{secondTurn},:);
                    tmpFlag = LeffTest(vehTgs, vehicle(LCIdx,:), leff);
                    if rand<P && tmpFlag%Lane Changed
                        if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                        [vehCount2,totVehCount,laneVIdx,vehicle,D,nextLane] = LC(vehCount2,totVehCount,laneVIdx,vehicle,D,nextLane,LCIdx,theLaneIdx,theTurn,laneIDList,frameBuff,linkGraph,linkLen);
                        Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );
                    end
                end
                if abs(P-1)<1e-2 && ~tmpFlag
                    vehicle(LCIdx,2) = 2;%waiting flag
                end
            else%DLC
                vehTgs = vehicle(laneVIdx{firstTurn},:);
                if rand<Pr && LeffTest(vehTgs, vehicle(LCIdx,:), leff)%Lane Changed
                    [vehCount1,totVehCount,laneVIdx,vehicle,D,nextLane] = LC(vehCount1,totVehCount,laneVIdx,vehicle,D,nextLane,LCIdx,theLaneIdx,firstTurn,laneIDList,frameBuff,linkGraph,linkLen);
                    Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );
                else%DLC of other lane
                    if twoTL
                        vehTgs = vehicle(laneVIdx{secondTurn},:);
                        if rand<Pr2 && LeffTest(vehTgs, vehicle(LCIdx,:), leff)%Lane Changed
                            [vehCount2,totVehCount,laneVIdx,vehicle,D,nextLane] = LC(vehCount2,totVehCount,laneVIdx,vehicle,D,nextLane,LCIdx,theLaneIdx,secondTurn,laneIDList,frameBuff,linkGraph,linkLen);
                            Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );
                        end
                    end
                end
            end
            LCVehs(IdxOFIdx) = [];
        end
    end
end
end

function result = LeffTest(vehTgs, vehLCIdx, leff)
if isempty(vehTgs)
    result = true;
else
    idx_tgs_front = find(vehTgs(:,4)>vehLCIdx(4));
    idx_tgs_rear = find(vehTgs(:,4)<=vehLCIdx(4));
    
    if ~isempty(idx_tgs_front)
        [~,idx_front] = min(vehTgs(idx_tgs_front,4));
        a = vehTgs(idx_front,4)-vehLCIdx(4)>leff;
    else
        a = true;
    end
    
    if ~isempty(idx_tgs_rear)
        [~,idx_rear] = max(vehTgs(idx_tgs_rear,4));
        b = vehLCIdx(4)-vehTgs(idx_rear,4)>leff;
    else
        b = true;
    end
    
    result = a&b;
end
end

function p = MLC(s,len)
    buff = 20;%m
    if len > buff
        p = (min(s,len-buff))/(len-buff);
    else
        p = 1;
    end
end