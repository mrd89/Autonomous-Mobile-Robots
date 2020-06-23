function[q] = integrateOdom( qInit , d, phi)
% takes in odom and pose and outtputs future pose
%
% INPUTS
%       qInit      	3-by-1 pose vector in global coordinates (x,y,theta)
%       d         	    list of distances robot will travel 1-by-K vector
%       phi          	list of angles robot will turn 1-by-K vector
%
% OUTPUTS
%       q           Robot pose after odometry


% init variables
n = length(d); % length of input data
q = zeros(3,n);
q = [qInit,q];

% decompose variables for readability
x = q(1,1);
y = q(2,1);
theta = q(3,1);


%loop through length of data
for i = 1:n 
    
    % current distance and angle measurements
    d_curr = d(i); 
    phi_curr = phi(i);
    
    %if phi =0; edge case
    if (phi_curr)==0 
        
        % calculate displacements given info above
        dx = d_curr*cos(theta);
        dy = d_curr*sin(theta);
        dtheta = phi_curr;
        
        % Current Configuration
        x = x + dx; 
        y = y + dy; 
        theta = theta + dtheta;
        q(:,i+1) = [x;y;theta];
        
    %phi != 0
    else 
        % calulate displacements ; phi != 0
        dx = 2 * d_curr / phi_curr * sin(phi_curr / 2) * cos(phi_curr / 2 + theta);
        dy = 2 * d_curr / phi_curr * sin(phi_curr / 2) * sin(phi_curr / 2 + theta);
        dtheta = phi_curr;
        
        %integrated pose of robot
        x = x + dx;
        y = y + dy;
        theta = theta + dtheta;
        
        q(:,i+1) = [x;y;theta];
    end
end
%output future poses
q = q(:,2:end);


end %end function
