function output = dijkstra1m(linkGraph,O,Ds)
m = length(Ds);
cost = Inf*ones(m,1);
for i = 1:m
    cost(i) = dijkstra(linkGraph,O,Ds(i));
end
output = min(cost);
end