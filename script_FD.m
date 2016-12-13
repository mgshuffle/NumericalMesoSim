%setting
linkLen = 600;
varargin = {linkLen,floor(600/6)};
interval = 60;
dir = './';
num = 5;

%initial
headline = [];
ldt = [];

for i = 1:num
    load([dir 'record_' num2str(i)]);
    data = [headline; data];
    data2 = data(data(:,8)~=1,:);%kill virVeh
    
    data2 = organizeRawData(data2);
    
    loopdata = loopdetect(data2,varargin);
    ldt = [ldt; loopdata];
    
    headline = data(end,:);
end

ald = aggregate(ldt,interval);

figure
plot(ald(:,6)*1000, ald(:,4)*3600,'.')