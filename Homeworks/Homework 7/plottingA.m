%% Q1

map = dlmread('hw7Sphere.txt');
figure
hold on
bound = plotSphereMap(map,100);
xlabel('x')
ylabel('y')
title('Sphere World');
legend([bound(1) bound(2)],'Boundary','Obstacles')




%% Q2

clear

map = dlmread('hw7Sphere.txt');



goal = [30,40];

C_att = 2;

C_rep = 1;

Q = 1;

maxWeight = 200;

potentialPlot( map, goal, C_att, C_rep, Q,maxWeight)
 
 
%% Local Min
 

clear

map = dlmread('hw7Sphere.txt');



goal = [30,40];

C_att = .01;

C_rep = .2;



Q = 100000;

maxWeight = 1;

potentialPlot( map, goal, C_att, C_rep, Q,maxWeight)



%% holonomic boi
clear


clear

map = dlmread('hw7Sphere.txt');



goal = [30,40];

C_att = 2;

C_rep = 7;

Q = 6;

maxWeight = 200;

%[potField,gradField,X,Y]=potentialMap( map, goal, C_att, C_rep, Q,maxWeight);


figure
hold on
bound = plotSphereMap(map,100);
xlabel('x')
ylabel('y')
title('Sphere World');
%legend([bound(1) bound(2)],'Boundary','Obstacles')
found =0;
pose = [80,55];
prevx = 80;
prevy = 55;
i=1;
while ~found
    
    [potField,gradField] = potentialPoint( map, goal, C_att, C_rep, Q, pose, maxWeight);
    
    pose(1) = pose(1) - gradField(1) / 800;
    pose(2) = pose(2) - gradField(2) / 800;
    
    
    traj= plot( [prevx pose(1)] , [prevy pose(2)], 'g-');
    
    if norm(pose-goal)<.5
        found =1;
    end
    i=i+1;
    
    prevx = pose(1);
    prevy = pose(2);
    
    if mod(i,50)==0
        k=1;
    end
    
end

   gh =  plot(goal(1),goal(2),'r*');
   sh = plot(80,55,'b*');
   
   legend([bound(1) bound(2),traj,gh,sh],'Boundary','Obstacles','Trajectory','Goal','Start')
    
    
    %% test running test program lul
    
    
    
    map = dlmread('hw7Sphere.txt');



goal = [30,40];

c_att = 2;

c_rep = 7;

Q = 6;

maxWeight = 200;

N = 40;
%finds min and max x,y coords
xmin = map(1,1) - map(1,3);
xmax = map(1,1) + map(1,3);
ymin = map(1,2) - map(1,3);
ymax = map(1,2) + map(1,3);


%mesh grid formed here
xpoints = linspace(xmin,xmax,N);
ypoints = linspace(ymin,ymax,N);

x = [1,2,3,4];
y = [30,50,60,80];
points = [xpoints',ypoints'];
    
    
    
    
    
    
    TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    %% simulation plot
    clear
    global dataStore
    
    N = length(dataStore.truthPose(:,1));
    
    figure

    hold on
   
    for i=1:N-1
        
        pose = dataStore.truthPose(i,2:3);
        posen = dataStore.truthPose(i+1,2:3);
        traj = plot( [pose(1) posen(1)],[pose(2) posen(2)],'g-');
    end
    
    obHandle = plotMapMatt(dlmread('hw7aMyMap.txt'));
    

    %xlim([-6 6])
    ylim([-6 6])
    axis equal
    
    xlabel('x (m)');
    ylabel('y (m)');
    
    legend( [traj,obHandle], 'Trajectory of Robot','Map Obstacles')
    