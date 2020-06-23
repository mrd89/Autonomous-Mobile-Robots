function pset = PF_live(pset,odom,measurements,odomFunc,depthFunc,Q,Noise,map)
%function takes in a previous particle set with odom, measurement
%functions, noise, and the map to output a new particle set
%
%   INPUTS
%       pset            Previous particle set
%       odom            odometry inputs
%       measurements    depth measurements
%       odomFunc        Anon func for integrade odom
%       depthFunc       Anon func for depth and range
%       Q               Measurement Noise
%       Noise           Particle Set Noise
%       Map             Map of the map
%   OUTPUTS
%         pset          Pset, N-by-4,  N-by-[x,y,theta,weight]
%
% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu


%find number of particles
N = length(pset(:,1));

%find dim of map
xmin = min( [ map(:,1) ; map(:,3) ] );
xmax = max( [ map(:,1) ; map(:,3) ] );
ymin = min( [ map(:,2) ; map(:,4) ] );
ymax = max( [ map(:,2) ; map(:,4) ] );

%iniate new pset
tempPset = zeros(N,4);

%add a normal noise to particles
pset(:,1:3) = pset(:,1:3) + random('normal',0,sqrt(Noise),[N,3]);

% Obtain all weights from helper function
weights =  particleWeight(pset,measurements,depthFunc,Q,map);

%normalize factor
normFactor = sum(weights);

if normFactor <10^(-15)
    weights(:,:) = 1/N;
else
    %normalize weights
    weights = weights ./ normFactor;
end

for i=1:N
    
    %get predicted position of robot
    pred_pose = odomFunc(pset(i,1:3)',odom(1),odom(2));

    %output new pset 
    tempPset(i,:) = [pred_pose',weights(i)];
    
    
end %end for i:N all particles

%pset(:,:) = tempPset(:,:);



%random sample all particles
for i=1:N
    
%    use randsample to get index of new pset
   index = randsample(N,1,true,weights);
    
   % use index to create new pset
   pset(i,:) = tempPset(index,:);
    
end %end final loop through pset

end %end of function



