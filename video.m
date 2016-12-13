dir = './';
num = 5;
mode = 1;%0:render only

feet2meter = 0.3048;

if mode == 1
    myVideo = VideoWriter('20161213.mp4','MPEG-4');
    myVideo.FrameRate = 20; %default 30
    myVideo.Quality = 50; %defaulet 100
    open(myVideo);
end

gcf = figure('Position', [100, 100, 1000, 190]);

for k = 1:num
    load([dir 'record_' num2str(k)]);

    a = data(1,end);
    b = data(end,end);
    
    %tmp Change
    data(:,3) = data(:,3) - 100;
    data(data(:,3)==4,3)=1;
    data(data(:,3)==5,3)=2;
    data(data(:,3)==6,3)=3;

    for i=a:b
        %draw network   
        hold on
        ylim([-22 0])
        xlim([0 600])
        for l = 1:6
            plot(linspace(0,600,100),(1-l)*12*feet2meter*ones(1,100),'k')
        end
        
        %draw vehs
        t=find(data(:,13)==i);
        if ~isempty(t)
            allVeh = data(t,1:11);
            idxReal = find(allVeh(:,8)==0 & allVeh(:,9)==0);
            idxLC = find(allVeh(:,8)==0 & allVeh(:,9)>0);
            idxVirtual = setdiff((1:length(t))',union(idxReal,idxLC));
            %dot
            plot(allVeh(idxReal,4),(6-allVeh(idxReal,3)*12)*feet2meter,'ob')
            plot(allVeh(idxLC,4),(6-allVeh(idxLC,3)*12)*feet2meter,'ok')
%             plot(allVeh(idxVirtual,4),(6-allVeh(idxVirtual,3)*12)*feet2meter,'or')
            text(600,0,num2str(i))
            %line
%             if ~isempty(idxReal)
%                 for v1 = 1:length(idxReal)
%                     quiver(allVeh(idxReal(v1),4)-5,(6-allVeh(idxReal(v1),3)*12)*feet2meter,5,0,1,'b')
%                 end
%             end
%             if ~isempty(idxLC)
%                 for v2 = 1:length(idxLC)
%                     quiver(allVeh(idxLC(v2),4)-5,(6-allVeh(idxLC(v2),3)*12)*feet2meter,5,0,1,'k')
%                 end
%             end
%             if ~isempty(idxVirtual)
%                 for v3 = 1:length(idxVirtual)
%                     quiver(allVeh(idxVirtual(v3),4)-5,(6-allVeh(idxVirtual(v3),3)*12)*feet2meter,5,0,1,'r')
%                 end
%             end

        else
            warning('No Vehs')
        end
        
        if mode == 1
            g=getframe(gcf);
            writeVideo(myVideo, g);
        end
        
        clf
    end

end



if mode == 1
    close(myVideo);
end