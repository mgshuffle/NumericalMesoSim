load('result20161104')
a=1;
b=1000;

feet2meter = 0.3048;
FRAME = record(:,12)*10;

myVideo = VideoWriter('record.mp4','MPEG-4');
myVideo.FrameRate = 20; %default 30
myVideo.Quality = 50; %defaulet 100
open(myVideo);

gcf = figure('Position', [100, 100, 1000, 190]);
for i=a:b
    %draw network   
    hold on
    ylim([-30 0])
    xlim([0 500])
    for l = 1:8
        plot(linspace(0,500,100),(1-l)*12*feet2meter*ones(1,100),'k')
    end
    
    %draw vehs
    t=find(abs(FRAME-i)<1e-3);
    if ~isempty(t)
        allVeh = record(t,1:11);
        idxReal = find(allVeh(:,8)==0);
        idxVirtual = setdiff((1:length(t))',idxReal);
        plot(allVeh(idxReal,4),(6-allVeh(idxReal,3)*12)*feet2meter,'ob')
        plot(allVeh(idxVirtual,4),(6-allVeh(idxVirtual,3)*12)*feet2meter,'or')
    else
        warning('No Vehs')
    end
    
    g=getframe(gcf);
    writeVideo(myVideo, g);
    
    clf
end

close(myVideo);