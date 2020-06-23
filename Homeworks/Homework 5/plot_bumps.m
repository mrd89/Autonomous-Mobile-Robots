%% plot map /bumps to see shit

map = load('loopMap.mat');
map = map.loopMap;

figure
hold on
map=plotMap(map); %plot map

%plot bump locations
for i=1:length(dataStore.bump(:,1))
    if (sum(dataStore.bump(i,2:7))>=1)
        h1=plot(dataStore.truthPose(i,2),dataStore.truthPose(i,3),'g*','DisplayName','Bump Readings');
    end
end
%plot trajectory
h2=plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'b-','DisplayName','Trajectory');
xlabel('X axis (m)');
ylabel('Y axis (m)');
title('Plot of Robot Trajectory and Bump Readings');
legend([h1 h2 map],{'Bump Readings','Trajectory','Map'})


hold off

%% NEXT BULLSHIT


