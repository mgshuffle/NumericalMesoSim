function nextLane = GetNextLane(O,D,linkGraph)
nextLane = zeros(length(O),1);
if length(O)==1
    e = zeros(length(D),1);
    L = zeros(length(D),1);
    for k = 1:length(D)
        [e(k),L_] = dijkstra(linkGraph,O,D(k));
        if length(L_)>1
            L(k) = L_(end-1);
        else
            L(k) = L_(end);
        end
    end
    [~,idx] = min(e);
    nextLane = L(idx);
else
    for i = 1:length(O)
        e = zeros(length(D{i}),1);
        L = zeros(length(D{i}),1);
        for k = 1:length(D{i})
            [e(k),L_] = dijkstra(linkGraph,O(i),D{i}(k));
            if length(L_)>1
                L(k) = L_(end-1);
            else
                L(k) = L_(end);
            end
        end
        [~,idx] = min(e);
        nextLane(i) = L(idx);
    end
end
end