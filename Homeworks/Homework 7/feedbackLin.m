function [v,w] = feedbackLin(vx,vy,theta, epsilon)
%This function transofmr Vx and Vy as inputs and calculates the V and
%omega for the robot to use in its control. 


%feedback linearize the function. Chose to not use matrix multiplication
%for simplicity in my code
% transformation matrix
rbi = [cos(theta) sin(theta); -sin(theta) cos(theta)];

% calculate the fwd and ang velocities and return them.
velocities = [1 0; 0 1/epsilon]*rbi*[vx; vy];
v = velocities(1);
w = velocities(2);
end

