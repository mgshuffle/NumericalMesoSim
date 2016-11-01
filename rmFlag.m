function vehicle=rmFlag(vehicle,idx_flag)
[idx2,idx1] = YinX2(vehicle(idx_flag,1),vehicle(idx_flag,6));
[~,nonHead] = YinX2(idx1,idx2);
head = setdiff(idx1,nonHead);
while ~isempty(head)
	Idx2Lead = head(1);
	lead = find(vehicle(:,1)==vehicle(Idx2Lead,6));

	Idx2Follow = find(idx1(nonHead)==idx2(Idx2Lead));
	if isempty(Idx2Follow)
		follow = find(vehicle(:,1)==vehicle(Idx2Lead,7));
	else
		while ~isempty(Idx2Follow)
			Idx2Follow = find(idx1(nonHead)==idx2(Idx2Follow));
		end
		follow = find(vehicle(:,1)==vehicle(Idx2Follow,7));
	end

	if vehicle(lead,6)~=0
		vehicle(vehicle(:,1)==vehicle(lead,6),7)=vehicle(follow,7);
	end

	if vehicle(follow,7)~=0
		vehicle(vehicle(:,1)==vehicle(follow,7),6)=vehicle(lead,6);
	end

	head(1)=[];
end
end