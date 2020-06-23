function HW7btest(WorkspaceFileName,BoundaryTopRightCorner,n_PRM,q_start,q_goal,radius)
% Test function for HW 7b
%
% Inputs:  WorkspaceFileName = file name for text file representing the obstacles in the workspace
%          BoundaryTopRightCorner = 2x1 vector capturing the [x; y] coordinates of the top right 
%                                   corner of the workspace. Assume the bottom left corner is [0; 0]
%          q_start = 2x1 vector of start point 
%          q_goal  = 2x1 vector of goal point 
%          radius  = robot radius in m





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUILD RRT POINT IS THE CONTROL WHICH PLOTS THE WHOLE TREE, IF YOU WANT TO
% HAVE IT PLOT, YOU NEED TO SET THE VARIABLE PLOTTING TO 1
% I HAVE IT SET TO THAT, SO IF ITS AN ISSUE FOR OTHER TESTING PLEASE CHANGE
% TO 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



map = dlmread('WorkspaceFileName');

mapsize = [0,BoundaryTopRightCorner(1),0,BoundaryTopRightCorner(2)];

%set as 5 meters for eff
delta = 5;

vart = buildRRT(map,mapsize,q_start,q_goal,delta,radius);
%vart = buildRRTpoint(map,mapsize,pose,goal,delta);


plotMap('WorkspaceFileName',[0,0,BoundaryTopRightCorner(1),BoundaryTopRightCorner(2)])

pathHandle = plot(vart(:,1),vart(:,2),'r');

legend([pathHandle],'RRT Path Found')

xlabel('x (m)')
ylabel('y (m)')

end

