close all
%% this version plots all but covariance, quick boioi
plotOpts = [{'color'},{'g'},{'linestyle'},{'-'},{'linewidth'},{1}];
    fig5 = figure(5);
    figure(5); hold on;
    plotMap(map);
    title('Plot of Depth EKF (Green is Cov, Red is actual pose, blue is mu ');
    xlabel('Global Coordinate X (m)');
    ylabel('Global Coordinate Y (m)');

    

    %plot(xt(1),xt(2),'r*');
    x=dataStore.truthPose(:,2);
    y=dataStore.truthPose(:,3);
    plot(x,y,'r*','MarkerSize',3,'DisplayName','Truth Pose');
    xt = dataStore.EKFmU;
    plot(xt(1,:),xt(2,:),'db','MarkerSize',3,'DisplayName','Beacon 2')
    
    
for i=1:length(dataStore.EKFmU-1)
    xt = dataStore.EKFmU(1:3,i);
    uh=(3*i-2);
    wut=(3*i-1);
    Sig = dataStore.EKFsIGMA(1:2,uh:wut);
    p(i)=plotCovEllipse(xt(1:2),Sig,[1],plotOpts,fig5);

end

% %% this plots slowly
% 
% 
% plotOpts = [{'color'},{'b'},{'linestyle'},{'-'},{'linewidth'},{1}];
% 
% for i=1:1:length(dataStore.EKFmU)
% 
%     fig5 = figure(5);
%     figure(5); hold on;
%     plotMap(map);
%     title('testing ');
%     xlabel('Global Coordinate X (m)');
%     ylabel('Global Coordinate Y (m)');
% 
%     xt = dataStore.EKFmU(:,i);
%     uh=(3*i-2);
%     wut=(3*i-1);
%     Sig = dataStore.EKFsIGMA(1:2,uh:wut);
%     plot(xt(1),xt(2),'r*');
%     dxt = xt(1)+cos(xt(3))/6;
%     dyt = xt(2) + sin(xt(3))/6;
%     plot([xt(1),dxt],[xt(2),dyt])
%     x=dataStore.truthPose(i,2);
%     y=dataStore.truthPose(i,3);
%     dx = x+cos(dataStore.truthPose(i,4))/5;
%     dy = y + sin(dataStore.truthPose(i,4))/5;
%     plot(dataStore.truthPose(i,2),dataStore.truthPose(i,3),'b*');
%     plot([x,dx],[y,dy]);
%     
%    % plotCovEllipse(xt(1:2),Sig,[1,2,3],plotOpts,fig5);
%    %hold off;
%    %close;
%     
%     
% end


%% next part

Num=20;

pset=zeros(Num,3);

x = linspace(-5,0,4);
y = linspace(-5,5,5);

px = zeros(5,4);
py=px;

for i=1:5
    px(i,:)=x;
end
for i=1:4
    py(:,i)=y;
end

for i=1:5
    for j=1:4
        dx - x(i,j)
        plot(



