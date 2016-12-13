%% organizeRawData: order the raw trajactory data
function [newData] = organizeRawData(data)

[m,n] = size(data);
newData = zeros(m,n);
count = 0;

[uniH,~] = unique(data(:,1));

for k = 1:length(uniH)
    newRows = data(data(:,1)==uniH(k),:);
    rowNum = length(newRows(:,1));
    [~,idxOrder] = sort(newRows(:,13));
    newRows = newRows(idxOrder,:);
    newData(count+1:count+rowNum,:) = newRows;
    count = count + rowNum;
end

end