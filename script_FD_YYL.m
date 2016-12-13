
p = floor(t/60)+1;
[q,~]=hist(p,1:max(p));

V(1)=harmmean(v(p==1));
plot(q*60/6./(V*3.6),q*60/6,'.')