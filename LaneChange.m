%vehicle: #1_VID #2_State(arrived==1) #3_laneID #4_S #5_Vel #6_leading #7_following #8_type(virtual==1) #9_(adjust_time_)remain(>=0) #10_space #11_headway0
function [vehicle,totVehCount] = LaneChange(vehicle,CellIdx,laneCount,frameBuff,totVehCount,linkLen)
%addpath(genpath(pwd));%
%ForcedLCIdx = find(vehicle(:,))%forcedLC

DenMax = 1/6;%veh/meter

[uniCIdx,~] = unique(CellIdx);
for k = 1:length(uniCIdx)
    theCell = find(CellIdx==uniCIdx(k));
    theCell = theCell(vehicle(theCell,8)~=1);%
    theCell = theCell(vehicle(theCell,9)==0);%
    if ~isempty(theCell)
        tmpIdx = theCell(vehicle(theCell,4)==max(vehicle(theCell,4)));
        theLaneID = vehicle(theCell(1),3);
        head = vehicle(tmpIdx,4) + vehicle(tmpIdx,10);
        tail = min(vehicle(theCell,4));
        if theLaneID==1
            vehCountL = ceil((head-tail)*DenMax);
        else
            idx_VehsL = find(vehicle(:,3)==theLaneID-1);
            idx_VehsL = idx_VehsL((vehicle(idx_VehsL,4)-head).*(vehicle(idx_VehsL,4)-tail)<=0);
            vehCountL = length(idx_VehsL);
        end
        if theLaneID==laneCount
            vehCountR = ceil((head-tail)*DenMax);
        else
            idx_VehsR = find(vehicle(:,3)==theLaneID+1);
            idx_VehsR = idx_VehsR((vehicle(idx_VehsR,4)-head).*(vehicle(idx_VehsR,4)-tail)<=0);
            vehCountR = length(idx_VehsR);
        end
        if vehCountL>vehCountR
            firstTurn = -1;
            vehCount1 = vehCountL;
            vehCount2 = vehCountR;
        else
            firstTurn = 1;
            vehCount1 = vehCountR;
            vehCount2 = vehCountL;
        end
        
        thisVehCount = length(theCell);
        while length(theCell)>=1
            IdxOFIdx = max(ceil(rand*length(theCell)),1);
            LCIdx = theCell(IdxOFIdx);
            
            Pr = max(0,(thisVehCount-vehCount1)/(head-tail)/DenMax);
            if rand<Pr%Lane Changed
                vehicle(LCIdx,9) = frameBuff;
                vehCount1 = vehCount1 + 1;
                totVehCount = totVehCount + 1;
                newVirVeh = vehicle(LCIdx,:);
                newVirVeh([1 3 8 9]) = [totVehCount theLaneID+firstTurn 1 frameBuff];
                idx_tgs = find(vehicle(:,3)==theLaneID+firstTurn);
                
                idx_tgs_front = idx_tgs(vehicle(idx_tgs,4)>vehicle(LCIdx,4));
                idx_tgs_rear = idx_tgs(vehicle(idx_tgs,4)<=vehicle(LCIdx,4));
                
                if isempty(idx_tgs_front)
                    newVirVeh(6)=0;
                    newVirVeh(10) = linkLen - newVirVeh(4);
                else
                    [~,tmp] = min(vehicle(idx_tgs_front,4));
                    idx_front = idx_tgs_front(tmp);
                    newVirVeh(6) = vehicle(idx_front,1);
                    vehicle(idx_front,7) = newVirVeh(1);
                    newVirVeh(10) = vehicle(idx_front,4) - newVirVeh(4);
                end
                newVirVeh(11) = newVirVeh(10)/newVirVeh(5);
                
                if isempty(idx_tgs_rear)
                    newVirVeh(7)=0;
                else
                    [~,tmp] = max(vehicle(idx_tgs_rear,4));
                    idx_rear = idx_tgs_rear(tmp);
                    newVirVeh(7) = vehicle(idx_rear,1);
                    vehicle(idx_rear,6) = newVirVeh(1);
                    vehicle(idx_rear,10) = newVirVeh(4)-vehicle(idx_rear,4);
                    vehicle(idx_rear,11) = vehicle(idx_rear,10) / vehicle(idx_rear,5);
                end
                vehicle = [vehicle;newVirVeh];
                
            else%trying the other Lane
                Pr2 = (thisVehCount-vehCount2)/(head-tail)/DenMax;
                if rand<Pr2%the other Lane Changed
                    vehicle(LCIdx,9) = frameBuff;
                    vehCount2 = vehCount2 + 1;
                    totVehCount = totVehCount + 1;
                    newVirVeh = vehicle(LCIdx,:);
                    newVirVeh([1 3 8 9]) = [totVehCount theLaneID-firstTurn 1 frameBuff];
                    idx_tgs = find(vehicle(:,3)==theLaneID-firstTurn);
                    
                    idx_tgs_front = idx_tgs(vehicle(idx_tgs,4)>vehicle(LCIdx,4));
                    idx_tgs_rear = idx_tgs(vehicle(idx_tgs,4)<=vehicle(LCIdx,4));
                    
                    if isempty(idx_tgs_front)
                        newVirVeh(6)=0;
                        newVirVeh(10) = linkLen - newVirVeh(4);
                    else
                        [~,tmp] = min(vehicle(idx_tgs_front,4));
                        idx_front = idx_tgs_front(tmp);
                        newVirVeh(6) = vehicle(idx_front,1);
                        vehicle(idx_front,7) = newVirVeh(1);
                        newVirVeh(10) = vehicle(idx_front,4) - newVirVeh(4);
                    end
                    newVirVeh(11) = newVirVeh(10)/newVirVeh(5);
                    
                    if isempty(idx_tgs_rear)
                        newVirVeh(7)=0;
                    else
                        [~,tmp] = max(vehicle(idx_tgs_rear,4));
                        idx_rear = idx_tgs_rear(tmp);
                        newVirVeh(7) = vehicle(idx_rear,1);
                        vehicle(idx_rear,6) = newVirVeh(1);
                        vehicle(idx_rear,10) = newVirVeh(4)-vehicle(idx_rear,4);
                        vehicle(idx_rear,11) = vehicle(idx_rear,10) / vehicle(idx_rear,5);
                    end
                    vehicle = [vehicle;newVirVeh];
                end
            end
            
            theCell(IdxOFIdx) = [];
        end
    end
end
end