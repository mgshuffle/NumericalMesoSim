feet2meter = 0.3048;

load('data_i80_1');
Frame_ID=data(:,2);
Local_Y=data(:,6);
v_Class=data(:,11);

%LocalY 不能有倒退点
s = testBackward(data);
if s>0 
    warning('backward warning');
end

SecBegin = 670;% feet
SecEnd = 1600;% feet

FrameBegin = min(Frame_ID(Local_Y>=SecEnd));
FrameEnd = max(Frame_ID(Local_Y<=SecBegin));

idx1 = find(Frame_ID>=FrameBegin & Frame_ID<=FrameEnd);
idx2 = find(Local_Y>=SecBegin & Local_Y<=SecEnd);
idx3 = find(v_Class~=1);
idx = intersect(idx1,idx2);
idx = intersect(idx,idx3);
newdata = data(idx,:);

newdata(:,6) = newdata(:,6) - SecBegin;
newdata(:,2) = newdata(:,2) - FrameBegin; 

idx4 = find(newdata(:,2)==0);
snapshot = [newdata(idx4,1) newdata(idx4,6)*feet2meter];
% csvwrite('snapshot.csv',snapshot);
snapData = newdata(idx4,:);
m = length(snapData(:,1));
vehicle = [(1:m)' zeros(m,1) snapData(:,14) snapData(:,6)*feet2meter snapData(:,12)*feet2meter...
          zeros(m,6)];
for la = 1:6
    thisLa = find(vehicle(:,3)==la);
    [~,order] = sort(vehicle(thisLa,4),'descend');
    thisLa = thisLa(order);
    vehicle(thisLa(2:end),6) = vehicle(thisLa(1:end-1),1);
    vehicle(thisLa(1:end-1),7) = vehicle(thisLa(2:end),1);
end
idx_f = find(vehicle(:,6)~=0);
idx_lv = vehicle(idx_f,6);
[IY,IX] = YinX2(vehicle(:,1),idx_lv);
idx_lv(IY) = IX;
vehicle(:,10) = (SecEnd-SecBegin)*feet2meter - vehicle(:,4);
vehicle(idx_f,10) = vehicle(idx_lv,4) - vehicle(idx_f,4);
vehicle(:,11) = vehicle(:,10)/vehicle(:,5);
%save('vehicle','vehicle');

[uniVID,idx5] = unique(newdata(:,1));
emitTime = newdata(idx5,2)/10;
uniVID = uniVID(emitTime>0);
emitTime = emitTime(emitTime>0);
[emitTime,idx6] = sort(emitTime);
uniVID = uniVID(idx6);
emitList = [emitTime uniVID];
% csvwrite('emitList.csv',emitList);
emitData = newdata(idx5,:);
emitData = emitData(emitData(:,2)>0,:);
emitData = emitData(idx6,:);
emitTable = [emitTime emitData(:,12)*feet2meter emitData(:,14)];
%save('emitTable','emitTable')


interval = 60;%seconds
loopdata = loopdetect(newdata,{984,328});
[AggLoopData] = aggregate(loopdata,interval);
