function [V,omega] = feedbackLin(Vxi,Vyi,pose,epsilon)
%This function transofmr Vx and Vy as inputs and calculates the V and
%omega for the robot to use in its control. 


%feedback linearize the function. Chose to not use matrix multiplication
%for simplicity in my code
V = Vxi * cos(pose(3)) + Vyi * sin(pose(3));
omega = 1/epsilon * Vxi*sin(pose(3)) - 1/epsilon * Vyi * cos(pose(3));

end

