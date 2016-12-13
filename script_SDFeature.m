vmax = 60/3.6;
vmin = 15/3.6;
kj = 0.115;
k = linspace(0,kj,1000);

a = 0.5:0.5:5;
b = 0.5:0.5:5;

figure
hold on
for i = 1:length(a)
    plot(k,SD1(k,[vmax vmin kj a(i) 1]));
end

figure
hold on
for i = 1:length(b)
    plot(k,SD1(k,[vmax vmin kj 1 b(i)]));
end