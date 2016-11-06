function emitTable = emitTableGenerater(ODList,vlim,laneCount)
	%ODIdx = ODList(:,1);
	demand = ODList(:,2);
	fTime = ODList(:,3);
	tTime = ODList(:,4);
	for i = 1:length(demand)
		time_ = rand(demand(i),1)*(tTime(i)-fTime(i)) + fTime(i);
		vel_ = rand(demand(i),1)*(vlim(2)-vlim(1)) + vlim(1);
		lanePos_ = max(1,ceil(rand(demand(i),1)*laneCount));
		emitTable = [emitTable; [time_ vel_ lanePos_]];
	end
end