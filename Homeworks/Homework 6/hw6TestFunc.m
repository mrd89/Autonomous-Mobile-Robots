function[] = hw6TestFunc(data)
% this function takes in a data set and outputs two different SLAM method
% plots
%
%   INPUTS
%       data        [m-by-n] each row = [x,y,theta,x1,y1...xn,yn]
%
%   OUTPUTS
% N/A

% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%% EKF SLAM


[mu,sigma,traj] = ekfSLAM(data);

%some plotting ops here
plotOpts = [{'color'},{'b'},{'linestyle'},{'-'},{'linewidth'},{1}];
fig5 = figure(1);
figure(1); hold on;


for i=4:4:19
    
    p1 = mu(i:i+1);
    p2 = mu(i+2:i+3);
    sig1 = sigma(i:i+1,i:i+1);
    i=i+2;
    sig2 = sigma(i:i+1,i:i+1);
hold on
    wallHandle1 = plot([p1(1), p2(1)],[p1(2),p2(2)],'-g');

        covHandle =  plotCovEllipse(p1',sig1,[1],plotOpts,fig5);
        covHandle = plotCovEllipse(p2',sig2,[1],plotOpts,fig5);
        hold on
end
hold on
tH = plot(traj(:,1),traj(:,2),'c-');
xlabel('x')
ylabel('y')
axis equal;

legend([ wallHandle1,tH],'Wall Locations','Robot Trajectory')

title('Fast SLAM Robot Mapping');
hold off

%% FastSLAM

%call function here
[pset,trajectory]= fastSLAM(data);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%some plotting ops here
plotOpts = [{'color'},{'b'},{'linestyle'},{'-'},{'linewidth'},{1}];
fig5 = figure(2);
figure(2); hold on;


for i=1:length(pset)
    weights(i) = pset.weight;
end

[~,maxIndex] = max(weights);
plot_particle = pset(maxIndex);


for oi =1:plot_particle.length/2
    
    p1 = plot_particle.marker(oi*2-1).mu;
    p2 = plot_particle.marker(oi*2).mu;
    sig1 = plot_particle.marker(oi*2-1).sig;
    sig2 = plot_particle.marker(oi*2).sig;
    
    if ~( isempty(p1) || isempty(p2) )
        for i=1:2
            for j=1:2
                if sig1(i,j)<10^(-10)
                    sig1(i,j) = 0;
                end
                if sig2(i,j)<10^(-10)
                    sig2(i,j) = 0;
                end
            end
        end
        
        
        

        covHandle =  plotCovEllipse(p1',sig1,[1],plotOpts,fig5);
        covHandle = plotCovEllipse(p2',sig2,[1],plotOpts,fig5);
    end
end



hold on
robHandle = plot(plot_particle.x,plot_particle.y,'r*');

for oi =1:plot_particle.length/2
    
    p1 = plot_particle.marker(oi*2-1).mu;
    p2 = plot_particle.marker(oi*2).mu;
    
    if ~( isempty(p1) || isempty(p2) )
        wallHandle = plot([p1(1), p2(1)],[p1(2),p2(2)],'-g');
    end
end

trajHandle = plot(trajectory(:,1),trajectory(:,2),'-k');
xlabel('x')
ylabel('y')
axis equal;

legend([robHandle, wallHandle,trajHandle],'Robot Final Position','Wall Locations','Robot Trajectory')

title('Fast SLAM Robot Mapping');

















