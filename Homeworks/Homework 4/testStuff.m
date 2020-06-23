
map = load('cornerMap.mat');
map = map.cornerMap;
angles = linspace(deg2rad(27),deg2rad(-27),9);

sensorOrigin= [0,0];
pose = [0,0,0]';


%uncomment wanted method, comment others out
method = 1; % EKF with GPS
%method = 2; % EKF with Depth
%method = 3; %PF with Depth

if (method ==1) %EKF with GPS
    
    R = .01 * eye(3);
    Q = .001 * eye(3);
    xt=[0;0;0];
    Sig = 10 * eye(3);
    hfunc = @(pose) hGPS(pose);
    Hfunc = @(pose) HjacGPS(map,pose,angles,sensorOrigin);
    sense_meas =@(pose) hGPS(gps);
    measurements = [1,4,1];

    %  [xt,Sig] = EKF(map,angles,measurements,SensorOrigin,Ht,xt,Sig,R,Q,d,phi);
    
    
elseif (method ==2) %EKF with Depth
    R = .01 * eye(3);
    Q = .001 * eye(9);
    %  Main_Func = @(Ht,xt,Sig,R,Q,d,phi) EKF(map,angles,measurements,SensorOrigin,Ht,xt,Sig,R,Q,d,phi);
    Hfunc = @(pose) HjacDepth(map,pose,angles,sensorOrigin);
    measurements = [1,1,1,1,1,1,1,1,1];
    sense_meas =@(pose) depth_and_range(pose,map,sensorOrigin,angles)

else %PF with Depth
    
    Q = .001;
    Hfunc = @(pose) HjacDepth(map,pose,angles,sensorOrigin);
    %~`~`~`~ TODO FIX THIS 
    %   Main_Func = @(pset,d,phi) PF(pset,d,phi,measurements,map,angles,Q);
end


Ht= Hfunc(pose);

d = .1;
phi = .05;
   if (method==1)
       sensor_measurements =pose; 
   elseif (method ==2)
       sensor_measurements = depth_and_range(pose,map,sensorOrigin,angles);
   end
   
 meas = pose;
    Hfunc = @(pose) HjacGPS(map,pose,angles,sensorOrigin);


[x,y]=EKF_NEW(pose,Sig,meas,Q,R,d,phi,hfunc,Hfunc,angles);


