function vehicle=rmFlag(vehicle,idx_flag)
[idx2,idx1] = YinX2(vehicle(idx_flag,1),vehicle(idx_flag,6));%idx1,2 is the index of idx_flag

idx_alone = setdiff(idx_flag, union(idx_flag(idx1),idx_flag(idx2)));
for i = 1:length(idx_alone)
    theIdx=idx_alone(i);
    if vehicle(theIdx,6)~=0
        vehicle(vehicle(:,1)==vehicle(theIdx,6),7)=vehicle(theIdx,7);
    end
    
    if vehicle(theIdx,7)~=0
        vehicle(vehicle(:,1)==vehicle(theIdx,7),6)=vehicle(theIdx,6);
    end
end

if ~isempty(idx1)    
    head = setdiff(idx1,idx2);
    while ~isempty(head)        
        theHead = head(1);        
        leadVID = vehicle(idx_flag(theHead),6);
        
        tmp = idx2(idx1==theHead);
        if isempty(tmp)
            theTail = theHead; 
        else
            while ~isempty(tmp)
                theTail = tmp;
                tmp = idx2(idx1==tmp);
            end
        end
        followVID = vehicle(idx_flag(theTail),7);
        
        if leadVID~=0
            vehicle(vehicle(:,1)==leadVID,7)=vehicle(idx_flag(theTail),7);
        end
        if followVID~=0
            vehicle(vehicle(:,1)==followVID,6)=vehicle(idx_flag(theHead),6);
        end
        
        head(1)=[];
    end
end

end