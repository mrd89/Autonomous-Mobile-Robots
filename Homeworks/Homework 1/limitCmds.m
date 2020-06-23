function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
%
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the
%   forward and angular velocity commands to drive a differential-drive
%   robot whose wheel motors saturate at +/- maxV.
%
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
%
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Daniel, Matthew



% find V of both wheels...
Vr = fwdVel + angVel * wheel2Center;
Vl = fwdVel - angVel * wheel2Center;

%find abs max of the two
if (abs(Vr)>abs(Vl))
    max = Vr;
else
    max = Vl;
end

% use max to see if greater than .5
if (abs(max) > maxV)
    %calculate ratio  of the max input and max v
    ratio = abs( maxV / max);
    
    % changes by the same ratio
    Vr = Vr *ratio;
    Vl = Vl *ratio;
end

% recalculate the wheel velocities :)
cmdV = (Vr+Vl) / 2;
cmdW = (Vl-Vr)/(wheel2Center*2);

end


