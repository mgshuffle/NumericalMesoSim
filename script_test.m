laneIDList = (101:106)';
m = length(laneIDList);
linkGraph = zeros(m,m);
r0 = [2; 3];
c0 = [5; 6];
linkGraph(r0+(c0-1)*m) = 1;
r1 = [1 2 2 3 5 6]';
c1 = [2 1 3 2 6 5]';
linkGraph(r1+(c1-1)*m) = 2;

turning = CalTuringLane(linkGraph);