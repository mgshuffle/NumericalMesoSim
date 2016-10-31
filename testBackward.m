function s = testBackward(data)
Vehicle_ID=data(:,1);

uniVeh = unique(Vehicle_ID);

s=0;
for i = 1:length(uniVeh)
    d = data(Vehicle_ID==uniVeh(i),:);
    Local_Y=d(:,6);
    s = length(find(diff(Local_Y)<0)) + s;
end
end