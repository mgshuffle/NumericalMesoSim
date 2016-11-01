function vehicle=rmFlag(vehicle,idx_flag)
[idx2,idx1] = YinX2(vehicle(idx_flag,1),vehicle(idx_flag,6));
[~,nonHead] = YinX2(idx1,idx2);
head = setdiff(idx1,nonHead);
while ~isempty(head)
	idx1(head(1))
end