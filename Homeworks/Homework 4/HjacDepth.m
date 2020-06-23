function Hdepth = HjacDepth(map,pose,angles,sensorOrigin)
% Creates a linearized matrix based on the pose and map

%   INPUTS
%       map            map data
%       pose          Posision in [x;y;theta]
%       angles         angles used in depth
%       sensorOrigin    sensor Origin [x,y]


%   OUTPUTS
%       mut             Mean at time t ( After one time step)


%set dx, also use dx as dy, dtheta
dx = 10^(-3);

% Initialize the matrix Hdepth
N = length(angles);
Hdepth = zeros(N,3);


%decompose position of robot
x = pose(1); 
y = pose(2);
theta = pose(3);

%calculate depth
range = rangePredict(pose,map,sensorOrigin,angles);
tru_depth = depthPredict(range,angles);

%find range of Pose + dx,dy,dtheta
range_dx = rangePredict([x+dx,y,theta],map,sensorOrigin,angles);
range_dy = rangePredict([x,y+dx,theta],map,sensorOrigin,angles);
range_dt = rangePredict([x,y,theta+dx],map,sensorOrigin,angles);

%calculate range based on that, Depth outputs [N,1] matrix
depth_dx = depthPredict(range_dx,angles);
depth_dy = depthPredict(range_dy,angles);
depth_dt = depthPredict(range_dt,angles);


%Finally linearize about the depth reading
Hdepth(:,1) = (depth_dx -tru_depth) / dx;
Hdepth(:,2) = (depth_dy -tru_depth) / dx;
Hdepth(:,3) = (depth_dt -tru_depth) / dx;
end
