%vehicle: #1_VID #2_State(arrived==1,waiting==2) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual>0) #9_(adjust_time_)remain(>=0) #10_space #11_headway0
%laneList: #1_ID #2_laneLen #3_laneStart 
function [vehicle,D,nextLane,totVehCount] = LaneChange3(vehicle,D,nextLane,totVehCount,CellIdx)
%addpath(genpath(pwd));%
%ForcedLCIdx = find(vehicle(:,))%forcedLC

global laneList
global linkGraph
global DenMax
global leff

laneIDList = laneList(:,1);
laneLen = laneList(:,2);
LPos = unique(laneList(:,4));
LPosCount = length(LPos);

%��LaneIDת����LanePosition������
vehLP = vehicle(:,3);
[IY,IX] = YinX2(laneIDList,vehLP);
vehLP(IY) = laneList(IX,4);
%����LanePos�������й���
for LPIdx = 1:LPosCount
    LPVehIdx{LPIdx} = find(vehLP==LPos(LPIdx));
end

turning = CalTuringLane(linkGraph);





[uniCIdx,~] = unique(CellIdx);
for k = 1:length(uniCIdx)
    %ȡ�����ڸ�cell��veh������theCell���Լ����пɱ��������veh������LCVehs
    theCell = find(CellIdx==uniCIdx(k));
    LCVehs = theCell(vehicle(theCell,8)==0 & vehicle(theCell,9)==0);%
    %�ó�����ID,��LaneList�е��������Լ�ת�򳵵�������
    theLaneID = vehicle(theCell(1),3);
    theLaneIdx = find(laneIDList==theLaneID);    
    theTurning = turning(theLaneIdx,:);
    
    %�����ڿɱ�������Ҹó�������ת�򳵵�
    if ~isempty(LCVehs) && sum(theTurning)>0 %LCV>1 and turnable lane count>0
        thisVehCount = length(theCell);%���ӳ�����
        tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));%ͷ��������λ����ǰ�ĳ�        
        head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);%�������α߽�
        tail = min(vehicle(theCell,4));%�������α߽�
        twoTL = theTurning(1)&theTurning(2);%flag �Ƿ���ڵڶ�ת�򳵵�

        CountMax = ceil((head-tail)*DenMax);
        if CountMax<1
            error('too short');
        end

        firstTurnPosIdx = find(LPos==laneList(theTurning(1),4));
        idx_FTurn = LPVehIdx{firstTurnPosIdx};
        
        idx_FArea = idx_FTurn((vehicle(idx_FTurn,4)-head).*(vehicle(idx_FTurn,4)-tail)<=0);
        vehCount1 = length(idx_FArea);
        Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );

        if twoTL
            secondTurnPosIdx = find(LPos==laneList(theTurning(2),4));
            idx_STurn = LPVehIdx{secondTurnPosIdx};
            
            idx_SArea = idx_STurn((vehicle(idx_STurn,4)-head).*(vehicle(idx_STurn,4)-tail)<=0);
            vehCount2 = length(idx_SArea);
            Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );

            if vehCount2<vehCount1%order
                tmpTurn = firstTurnPosIdx;
                firstTurnPosIdx = secondTurnPosIdx;
                secondTurnPosIdx = tmpTurn;

%                 idx_tmpT = idx_FTurn;
%                 idx_FTurn = idx_STurn;
%                 idx_STurn = idx_tmpT;
                
                idx_tmpA = idx_FArea;
                idx_FArea = idx_SArea;
                idx_SArea = idx_tmpA;

                vehCount1 = length(idx_FArea);
                vehCount2 = length(idx_SArea);
                Pr = max( 0, (thisVehCount-(vehCount1+1))/CountMax );
                Pr2 = max( 0, (thisVehCount-(vehCount2+1))/ceil((head-tail)*DenMax) );
            end
        end

        flagLC = false;
        while length(LCVehs)>=1            
            IdxOFIdx = max(ceil(rand*length(LCVehs)),1);
            LCIdx = LCVehs(IdxOFIdx);
            %LCVID = vehicle(LCIdx,1);
            if linkGraph(laneIDList==vehicle(LCIdx,3),nextLane(LCIdx))==2%MLC
                theTurn = nextLane(LCIdx);
                PMLC = MLC(vehicle(LCIdx,4),laneLen(theTurn));
                theTurnPos = laneList(theTurn,4);
                if theTurnPos==firstTurnPosIdx
                    P = min(1,PMLC + Pr);
                    vehTgs = vehicle(LPVehIdx{firstTurnPosIdx},:);
                    tmpFlag = LeffTest(vehTgs, vehicle(LCIdx,:), leff);
                    if (rand<P || vehicle(LCIdx,2)==2) && tmpFlag %Lane Changed
                        %debug
                        if vehicle(LCIdx,2)==2
                        end
                        if flagLC == false
                            flagLC = true;
                        else
                            warning('secend veh')
                        end
                        if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                        [vehicle,D,nextLane,vehCount1,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,vehCount1,totVehCount,LPVehIdx,LPos,LCIdx,theTurn);
                        Pr = max( 0, (thisVehCount-vehCount1)/CountMax );
                    end
                else
                    P = min(1,PMLC + Pr2);
                    vehTgs = vehicle(LPVehIdx{secondTurnPosIdx},:);
                    tmpFlag = LeffTest(vehTgs, vehicle(LCIdx,:), leff);
                    if (rand<P || vehicle(LCIdx,2)==2) && tmpFlag%Lane Changed
                        %debug
                        if vehicle(LCIdx,2)==2
                        end
                        if flagLC == false
                            flagLC = true;
                        else
                            warning('secend veh')
                        end
                        if vehicle(LCIdx,2)==2, vehicle(LCIdx,2)=0; end
                        [vehicle,D,nextLane,vehCount2,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,vehCount2,totVehCount,LPVehIdx,LPos,LCIdx,theTurn);
                        Pr2 = max( 0, (thisVehCount-vehCount2)/CountMax );
                    end
                end
                if abs(P-1)<1e-2 && ~tmpFlag
                    vehicle(LCIdx,2) = 2;%waiting flag
                end
            else%DLC
                vehTgs = vehicle(LPVehIdx{firstTurnPosIdx},:);
                if rand<Pr && LeffTest(vehTgs, vehicle(LCIdx,:), leff)%Lane Changed
                    if flagLC == false
                        flagLC = true;
                    else
                        warning('secend veh')
                    end
                    [vehicle,D,nextLane,vehCount1,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,vehCount1,totVehCount,LPVehIdx,LPos,LCIdx,theTurning(1));
                    Pr = max( 0, (thisVehCount-vehCount1)/CountMax );
                else%DLC of other lane
                    if twoTL
                        vehTgs = vehicle(LPVehIdx{secondTurnPosIdx},:);
                        if rand<Pr2 && LeffTest(vehTgs, vehicle(LCIdx,:), leff)%Lane Changed
                            if flagLC == false
                                flagLC = true;
                            else
                                warning('secend veh')
                            end
                            [vehicle,D,nextLane,vehCount2,totVehCount,LPVehIdx] = LC(vehicle,D,nextLane,vehCount2,totVehCount,LPVehIdx,LPos,LCIdx,theTurning(2));
                            Pr2 = max( 0, (thisVehCount-(vehCount2+1))/CountMax );
                        end
                    end
                end
            end
            LCVehs(IdxOFIdx) = [];
        end
    end
end
end

function result = LeffTest(vehTgs, vehLC, leff)
if isempty(vehTgs)
    result = true;
else
    idx_tgs_front = find(vehTgs(:,4)>vehLC(4));
    idx_tgs_rear = find(vehTgs(:,4)<=vehLC(4));
    
    if ~isempty(idx_tgs_front)
        [~,idx_front] = min(vehTgs(idx_tgs_front,4));
        a = vehTgs(idx_tgs_front(idx_front),4)-vehLC(4)>leff;
    else
        a = true;
    end
    
    if ~isempty(idx_tgs_rear)
        [~,idx_rear] = max(vehTgs(idx_tgs_rear,4));
        b = vehLC(4)-vehTgs(idx_tgs_rear(idx_rear),4)>leff;
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