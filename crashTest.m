function IDX = crashTest(vehicle)
idx_f = find(vehicle(:,6)~=0);
[IY,IX] = YinX2(vehicle(:,1),vehicle(idx_f,6));
vehicle(idx_f(IY),6)=IX;

IDX = idx_f(vehicle(idx_f,4)>=vehicle(vehicle(idx_f,6),4));
end