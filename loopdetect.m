%% loopdetect: function description
%format: %format: 1#Location 2#VID 3#FID(time) 4#LaneID 5#v_Vel(m/s) 6#headway 7#headwayGlobal(sectional)
function [records] = loopdetect(data,varargin)

	sampleFrequency = 10;%Hz
    if isempty(varargin)
        RdLen = 1700;%road length 1700 meter
        locBuffer = 100;%distance between meter
    else
        p = cell2mat(varargin{1});
        RdLen = p(1);
        locBuffer = p(2);
    end
	passTime = zeros(max(data(:,3)),floor(RdLen/locBuffer)+1);%initial passTime at Lane i(row) of Location j(column) is 0 laneIDNum*locationNum
	passTimeGlobal = zeros(1,floor(RdLen/locBuffer)+1);
	records = [];

	[~,idx] = sort(data(:,13));
	data = data(idx,:);
	[m,~] = size(data);

	
	[uniFID,heads] = unique(data(:,13));
	tails = [heads(2:end)-1; m];
    fnum = length(heads);

	for i = 1:fnum-1
		former = data(heads(i):tails(i),:);
		latter = data(heads(i+1):tails(i+1),:);
		[~,ia,ib] = intersect(former(:,1),latter(:,1));
		former = former(ia,:);
		latter = latter(ib,:);

		Location = (floor(former(:,4)/locBuffer) + 1) * locBuffer;

		judgeMat = (former(:,4)-Location).*(latter(:,4)-Location);
		idx_J = find(judgeMat<=0);
        if (~isempty(idx_J))
			LaneID = former(idx_J,3);
			loIdx = Location(idx_J)/locBuffer;
			timeNow = uniFID(i)/sampleFrequency;
			headway = zeros(length(idx_J),1);
			headwayGlobal = zeros(length(idx_J),1);
			for k = 1:length(idx_J)
                if LaneID(k)<=0 || loIdx(k)<=0
                end
				headway(k) = timeNow - passTime(LaneID(k),loIdx(k));
				passTime(LaneID(k),loIdx(k)) = timeNow;
				headwayGlobal(k) = timeNow - passTimeGlobal(loIdx(k));
				passTimeGlobal(loIdx(k)) = timeNow;
			end
			%fix v_vel==0
			idxZeroV = idx_J(former(idx_J,5)==0);
			former(idxZeroV,5) = (latter(idxZeroV,4)-former(idxZeroV,4))*sampleFrequency;
			%format: 1#Location 2#VID 3#FID(time) 4#LaneID 5#v_Vel(m/s) 6#headway 7#headwayGlobal(sectional)
			records = [records; Location(idx_J) former(idx_J,[1 13]) LaneID former(idx_J,5) headway headwayGlobal];
        end
	end
end
