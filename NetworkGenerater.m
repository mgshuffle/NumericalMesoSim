function [laneList,linkGraph,ETT] = NetworkGenerater()
global capacity

%所有道路可以通行
laneIDList = (101:106)';
laneList = [laneIDList 300*ones(6,1) [zeros(3,1);300*ones(3,1)] [1;2;3;1;2;3] [4;5;6;0;0;0]];


m = length(laneIDList);
linkGraph = zeros(m,m);
r1 = [1; 2; 3];
c1 = [4; 5; 6];
linkGraph(r1+(c1-1)*m) = 1;
r2 = [1 2 2 3 4 5 5 6]';
c2 = [2 1 3 2 5 4 6 5]';
linkGraph(r2+(c2-1)*m) = 2;

ETT = [104 capacity*0.5 0; 105 capacity 0;106 capacity 0];

end