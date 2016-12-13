data = record(1:min(find(record(:,1)==0))-1,:);
LP = data(:,3)-100;
LP(LP==5)=2;
LP(LP==6)=3;
for f = data(1,13):data(end,13)
    for p = 1:3
        allVeh = data(data(:,13)==f & LP==p, :);
        if ~isempty(allVeh)
            [~,idx] = sort(allVeh(:,4),'descend');
            allVeh=allVeh(idx,:);
            fwVID = [0;allVeh(1:end-1,1)];
            bwVID = [allVeh(2:end,1);0];
            fwErr = find(allVeh(:,6)~=fwVID);
            bwErr = find(allVeh(:,7)~=bwVID);
            if ~isempty(fwErr)
                disp(f)
                disp(p)
                disp(allVeh(fwErr,1))
            end
            if ~isempty(bwErr)
                disp(f)
                disp(p)
                disp(allVeh(bwErr,1))
            end
        end
    end
end