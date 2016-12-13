target = [41 23 9 13];
mode = 1;%0:render only

feet2meter = 0.3048;

if mode == 1
    myVideo = VideoWriter('20161124.mp4','MPEG-4');
    myVideo.FrameRate = 20; %default 30
    myVideo.Quality = 50; %defaulet 100
    open(myVideo);
end

gcf = figure('Position', [100, 100, 1000, 190]);

data = record(record(:,1)~=0,:);
a = data(1,end);
b = data(end,end);

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
%         idxReal = find(allVeh(:,8)==0 & allVeh(:,9)==0);
%         idxLC = find(allVeh(:,8)==0 & allVeh(:,9)>0);
%         idxVirtual = setdiff((1:length(t))',union(idxReal,idxLC));
%         plot(allVeh(idxReal,4),(6-allVeh(idxReal,3)*12)*feet2meter,'ob')
%         plot(allVeh(idxLC,4),(6-allVeh(idxLC,3)*12)*feet2meter,'ok')
%         plot(allVeh(idxVirtual,4),(6-allVeh(idxVirtual,3)*12)*feet2meter,'or')
%         idxTar = YinX2(target,allVeh(:,1));
%         plot(allVeh(idxTar,4),(6-allVeh(idxTar,3)*12)*feet2meter,'*')
    else
        warning('No Vehs')
    end
    
    if mode == 1
        g=getframe(gcf);
        writeVideo(myVideo, g);
    end
    
    clf
end



if mode == 1
    close(myVideo);
end