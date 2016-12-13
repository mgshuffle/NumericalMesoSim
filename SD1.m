function velEql = SD1(K,para)
if isempty(para)
    para = [50/3.6,15.6946188969802/3.6,114.313208506104/1000,2.21680315505852,9.98014217655979];
end
vMax = para(1);
vMin = para(2);
kJam = para(3);
a = para(4);
b = para(5);
if K<=kJam
    velEql = vMin + (vMax - vMin)*(1-(K/kJam).^a).^b;
else
    velEql = 0;
end
end