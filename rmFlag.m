function vehicle=rmFlag(vehicle,idx_flag)
[idx2,idx1] = YinX2(vehicle(idx_flag,1),vehicle(idx_flag,6));
% %debug
% disp('All flag')
% disp(vehicle(idx_flag,[6 1 7]))

idx_alone = setdiff(idx_flag, union(idx1,idx2));
% %debug
% disp('Alone')
% disp(vehicle(idx_flag,[6 1 7]))


for i = 1:length(idx_alone)
    theIdx=idx_alone(i);
    if vehicle(theIdx,6)~=0
        vehicle(vehicle(:,1)==vehicle(theIdx,6),7)=vehicle(theIdx,7);
    end
    
    if vehicle(theIdx,7)~=0
        vehicle(vehicle(:,1)==vehicle(theIdx,7),6)=vehicle(theIdx,6);
    end
    
%     %debug
%     if vehicle(theIdx,6)~=0
%         disp(vehicle(vehicle(:,1)==vehicle(theIdx,6),[1 7]))
%     end
%     if vehicle(theIdx,7)~=0
%         disp(vehicle(vehicle(:,1)==vehicle(theIdx,7),[6 1]))
%     end
end



if ~isempty(idx1)
    %debug
    disp('Coupled')
    disp([idx1 idx2])
    disp('All flag')
    disp(vehicle(idx_flag,[6 1 7]))

    [~,nonHead] = YinX2(idx1,idx2);
    disp(nonHead)
    head = setdiff(idx1,nonHead);
    while ~isempty(head)
%         %debug
%         disp('Head(s)')
%         disp(head)
        Idx2Lead = head(1);
        lead = find(vehicle(:,1)==vehicle(Idx2Lead,6));
        
        Idx2Follow = find(idx1==idx2(idx1==Idx2Lead));
        if isempty(Idx2Follow)
            follow = find(vehicle(:,1)==vehicle(Idx2Lead,7));
        else
            while ~isempty(Idx2Follow)
                Idx2Follow = find(idx1(nonHead)==idx2(Idx2Follow));
            end
            follow = find(vehicle(:,1)==vehicle(Idx2Follow,7));
        end
        
        %debug
        if vehicle(lead,6)~=0
            disp(vehicle(vehicle(:,1)==vehicle(lead,6),[6 1]))
        else
            disp('0 as leading')
        end
        if vehicle(follow,7)~=0
            disp(vehicle(vehicle(:,1)==vehicle(follow,7),[1 7]))
        else
            disp('0 as following')
        end
        
        if vehicle(lead,6)~=0
            vehicle(vehicle(:,1)==vehicle(lead,6),7)=vehicle(follow,7);
        end
        
        if vehicle(follow,7)~=0
            vehicle(vehicle(:,1)==vehicle(follow,7),6)=vehicle(lead,6);
        end
        
%         disp(vehicle(vehicle(:,1)==vehicle(lead,6),[6 1]))
%         disp(vehicle(vehicle(:,1)==vehicle(follow,7),[1 7]))
        head(1)=[];
    end
end


end