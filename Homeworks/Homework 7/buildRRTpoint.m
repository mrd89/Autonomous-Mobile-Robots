function [waypointMap] = buildRRTpoint(map,mapsize,pose,goal,delta)
% This function takes in a map and ouputs the roadmap covering Q_Free
%
% INPUT:
%       map         poligonal map input as matrix
%       mapsize     [xmin,xmax,ymin,ymax] for largest possible location
%       pose        [x,y] current position
%       goal        [x,y] goal position
%
% OUTPUT:
%       waypointMap  list of [x,y] coords in order from pose to goal


% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu

%set plotting to 0 or 1, 1=plotting
plotting = 1;

if plotting
    figure
    hold on
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% decompose obstacles into matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = length(map(:,1)); %number of obs
J = length(map(1,:)); %max num of vertices

k = 1 ;
for i=1:I %for obstacles
    for j=1:2:J %for nodes in obstacle
        x = map(i,j);
        y = map(i,j+1);
        
        if ~( x ==0 && y ==0)
            nodeMap(k,:) = [ i,x,y];
            plot(x,y,'*')
            k = k+1;
        end
    end
end



% length of map
dx = mapsize(2) - mapsize(1);
dy = mapsize(4) - mapsize(3);
if plotting
temp = load('cornerMap.mat');
temp = temp.map;
plotMapMatt(temp);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initiate cell/struct data structure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tree.node(1).x = pose(1);
tree.node(1).y = pose(2);
tree.node(1).prev = NaN;
tree.length = 1;


%it aint found yet
found =0;
while (~found)
    
    %random variable in the map
    rx = rand * dx;
    ry = rand * dy;
    
    

    %find nearest free-path node
    minDist = intmax;
    minIndex = NaN;
    
    for i=1:tree.length
        
        vect = [rx,ry]-[tree.node(i).x,tree.node(i).y];
        dist = norm(vect);
        
        if dist<minDist
            minDist = dist;
            minIndex = i;
        end %end if dist
    end %for i to length
    
    [hits,~] = isFreePath(nodeMap,[tree.node(minIndex).x,tree.node(minIndex).y],[rx,ry],20);
    
    [hit] = isinsidewall (nodeMap,[rx,ry]);
    
    %if new pose is within the map and within delta size and does not hit
    if dist<delta && (rx>=mapsize(1) && rx<= mapsize(2) ) &&((ry>=mapsize(3) && ry<=mapsize(4) )) && hits<1 && ~hit
        
        %add to length of the tree
        tree.length = tree.length +1;
        
        %keep track of parent node
        tree.node(tree.length).prev = minIndex;
        
        %set new branch
        tree.node(tree.length).x = rx;
        tree.node(tree.length).y = ry;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PLOTTING
        if plotting
            %nodeH = plot(rx,ry,'b*');
            lineH = plot([rx,tree.node(minIndex).x],[ry,tree.node(minIndex).y],'c-');
        end
        %END plotting
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %check if it hits an obstacle
        [hits,~] = isFreePath(nodeMap,goal,[rx,ry],60);
        %if within delta
        if hits<1
            found =1;
            foundIndex = tree.length;
        end
        
        
    else
        nx = tree.node(minIndex).x + vect(1)./dist .* delta;
        ny = tree.node(minIndex).y + vect(2)./dist .* delta;
        
        
        [hits,dist] = isFreePath(nodeMap,[tree.node(minIndex).x,tree.node(minIndex).y],[nx,ny],20);
        
        [hit] = isinsidewall (nodeMap,[nx,ny]);
        
        if hits<1 && (nx>=mapsize(1) && nx<= mapsize(2) ) &&((ny>=mapsize(3) && ny<=mapsize(4) )) && ~hit
            
            %add to length of the tree
            tree.length = tree.length +1;
            
            %keep track of parent node
            tree.node(tree.length).prev = minIndex;
            
            %set new branch
            tree.node(tree.length).x = nx;
            tree.node(tree.length).y = ny;
            
            %check if it hits an obstacle
            [hits,~] = isFreePath(nodeMap,goal,[nx,ny],60);
            %if within delta
            if hits<1
                found =1;
                foundIndex = tree.length;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % PLOTTING
            if plotting
                %nodeH = plot(rx,ry,'b*');
                lineH = plot([nx,tree.node(minIndex).x],[ny,tree.node(minIndex).y],'c-');
            end
            %END plotting
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end %enf if no hits
    end %if dist <delta
    
    
    %if 3000 nodes created, unlikely to find it with a larger number
    if tree.length >3000
        found =2; %error code
    end %end if length >
    
    
end %end while

if found ==2
    waypointMap = NaN;
else
    
    atStart =0;
    waypoints(1).pose = goal;
    
    i=foundIndex;
    l=1;
    while (~atStart)
        l=l+1;
        if isnan(tree.node(i).prev)
            atStart =1;
        else
            waypoints(l).pose = [tree.node(i).x,tree.node(i).y];
            i = tree.node(i).prev;
            
        end %if null
        
    end %while
    
    waypointMap = zeros(l-1,2);
    
    for j=1:l-1
        
        waypointMap(j,1:2) = waypoints(l-j).pose;
    end
%    legend([lineH],'Random Tree Path');
end% if found==2
end %function




function [hit] = isinsidewall (map,pose)
%check if within a wall
    
    hit = [];
    
    I = max(map(:,1));
    
    for k = 1:I
        
        index = find(map(:,1)==k);
        [hit] = hit+inpolygon(pose(2),pose(2),map([index],2),map([index],3));
        
    end %end for I
    
    if sum(hit) <1
        hit =0;
    else
        hit =1
    end
    


end

