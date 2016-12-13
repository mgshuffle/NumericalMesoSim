% ldt_=ldt(ldt(:,1)==100,:);
% [~,idx] = sort(ldt_(:,3));
% 
% t=ldt_(idx,3);
% 
% [a, b] = hist(t,0:1:max(t));
% 
% s = zeros(size(a));
% S = 0;
% 
% for i = 1:length(a)
%     S = S + a(i);
%     s(i) = S;
% end

%Yunlin code
[a, b] = hist(t,0:.1:max(t));

s = zeros(size(a));
S = 0;

for i = 1:length(a)
    S = S + a(i);
    s(i) = S;
end