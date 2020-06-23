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
%   Kitahara, Jasmine

tm = [0.5 0.5; 1/wheel2Center/2 -1/wheel2Center/2];
vel = tm\[fwdVel; angVel];
vr = vel(1); vl = vel(2);
if abs(vl) >= abs(vr) && abs(vl) > maxV
    s = maxV/abs(vl);
    vr = s*vr;
    vl = s*vl;
    cmdvs = tm*[vr; vl];
    cmdV = cmdvs(1); cmdW = cmdvs(2);
elseif abs(vr) >= abs(vl) && abs(vr) > maxV
    s = maxV/abs(vr);
    vr = s*vr;
    vl = s*vl;
    cmdvs = tm*[vr; vl];
    cmdV = cmdvs(1); cmdW = cmdvs(2);

else
    cmdV = fwdVel;
    cmdW = angVel;
end

end
