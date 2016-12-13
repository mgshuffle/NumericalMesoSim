%对单路段运行结果的微观分析
load('record_1')
data = data(data(:,8)==0,:);
data = organizeRawData(data);

[VID,idx] = unique(data(:,1));
vcount = length(idx);

% TrT = zeros(vcount,1);
% LC = zeros(vcount,1);
TrT = [];
LC = [];
speed = [];
endtimes = 0;
for i = 1:vcount
    theVeh = data(data(:,1)==VID(i),:);
    if 600 - theVeh(end,4)<1
        endtimes = endtimes +1;
        TrT = [TrT; theVeh(end,12)-theVeh(1,12)];
        LC = [LC;length(find(diff(theVeh(:,3))~=0))];
        speed = [speed;mean(theVeh(:,5))];
    end
end