feet2meter = 0.3048;

load('data_i80_2');
Frame_ID=data(:,2);
Local_Y=data(:,6);
v_Class=data(:,11);

%LocalY ä¸èƒ½æœ‰å?é€?‚¹
s = testBackward(data);
if s>0 
    warning('backward warning');
end

SecBegin = 670;% feet
SecEnd = 1600;% feet

FrameBegin = min(Frame_ID(Local_Y>=SecBegin));
FrameEnd = max(Frame_ID(Local_Y<=SecEnd));

idx1 = find(Frame_ID>=FrameBegin & Frame_ID<=FrameEnd);
idx2 = find(Local_Y>=SecBegin & Local_Y<=SecEnd);
idx3 = find(v_Class~=1);
idx = intersect(idx1,idx2);
idx = intersect(idx,idx3);
newdata = data(idx,:);

newdata(:,6) = newdata(:,6) - SecBegin;
newdata(:,2) = newdata(:,2) - FrameBegin; 

[uniVID,idx5] = unique(newdata(:,1));
emitTime = newdata(idx5,2)/10;
[emitTime,idx6] = sort(emitTime);
uniVID = uniVID(idx6);
idx5 = idx5(idx6);
emitList = [emitTime newdata(idx5,12)*feet2meter newdata(idx5,14)];
csvwrite('emitList.csv',emitList);