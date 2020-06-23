function [] = TestFunHW1(TestVals, maxVel, wheel2Center)
% Test function for MAE 4180/5180, homework 1.  This function runs the
% student's feedback linearization and limit commands functions for a given
% set of input data, and plots the resulting V-W velocities.
% Inputs:
%   TestVals - A Nx4 matrix, where each row represents a different set of
%              input test values: [velX, velY, Theta, Epsilon].
%              velX : x-component of desired velocity in inertial frame (m/s)
%              velY : y-component of desired velocity in inertial frame (m/s)
%              Theta : Robot's current orientation [rad]
%              Epsilon : Distance from (0,0) used for feedback linearization
%   maxVel - Desired maximum robot wheel velocities (m/s)
%   wheel2Center - Distance between wheels and center of robot (m)

% Pre-allocate matrices to store linearized and limited velocities:
linV = zeros(size(TestVals,1), 2);
limV = zeros(size(TestVals,1), 2);

% Loop through the input data
for i = 1:size(TestVals,1)
   
    % Grab the current set of data
    velX = TestVals(i, 1); % Desired X-velocity of the robot
    velY = TestVals(i, 2); % Desired Y-velocity of the robot
    Theta = TestVals(i, 3); % Current robot orientation
    Epsilon = TestVals(i, 4); % Feedback linearization distance, epsilon
    
    % STUDENTS: Insert call to your feedback linearization function, such
    % that the function outputs the following variables:
    %   cmdV - the desired forward velocity of the robot
    %   cmdW - the desired angular velocity of the robot
    [cmdV,cmdW] = feedbackLin(velX,velY,[0,0,Theta],Epsilon);

    
    % Store the linearized velocities in the matrix linV
    linV(i,1) = cmdV; linV(i,2) = cmdW;
    
    % STUDENTS: Insert call to your limit commands function, such that the
    % function outputs the following variables:
    %   cmdV - the desired forward velocity of the robot
    %   cmdW - the desired angular velocity of the robot
    [cmdV,cmdW] = limitCmds(cmdV,cmdW,maxVel,wheel2Center);

    
    % Store the limited command velocities
    limV(i,1) = cmdV; limV(i,2) = cmdW;
    
end

% Open a figure
figure
% Plot the forward velocities
subplot(2,1,1)
hold on
plot(linV(:,1)', 'b-s') % Plot linearized velocities
plot(limV(:,1)', 'r--o') % Plot limited velocities
plot([1,size(linV,1)], [0,0], 'k:') % Plot a dotted line at vel=0
xlabel('Input Number')
ylabel('Forward Velocity (m/s)')
legend('Linearized Velocity', 'Limited Velocity')
% Plot the angular velocities
subplot(2,1,2)
hold on
plot(linV(:,2)', 'b-s') % Plot linearized velocities
plot(limV(:,2)', 'r--o') % Plot limited velocities
plot([1,size(linV,1)], [0,0], 'k:') % Plot a dotted line at vel=0
xlabel('Input Number')
ylabel('Angular Velocity (rad/s)')

end