feet2meter = 0.3048;

load('data_i80_2');
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
csvwrite('snapshot.csv',snapshot);

[uniVID,idx5] = unique(newdata(:,1));
emitTime = newdata(idx5,2)/10;
uniVID = uniVID(emitTime>0);
emitTime = emitTime(emitTime>0);
[emitTime,idx6] = sort(emitTime);
uniVID = uniVID(idx6);
emitList = [emitTime uniVID];
csvwrite('emitList.csv',emitList);

interval = 60;%seconds
loopdata = loopdetect(newdata,{SecEnd-SecBegin,floor((SecEnd-SecBegin)/2)});
[AggLoopData] = aggregate(loopdata,interval);
