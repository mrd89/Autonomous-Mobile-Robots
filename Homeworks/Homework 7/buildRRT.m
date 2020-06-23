function [waypointMap] = buildRRT(map,mapsize,pose,goal,delta,robRad)
% This function takes in a map and ouputs the roadmap covering Q_Free
%
% INPUT:
%       map         poligonal map input as matrix
%       mapsize     [xmin,xmax,ymin,ymax] for largest possible location
%       pose        [x,y] current position
%       goal        [x,y] goal position
%       robRad      robot radius (m)
%
% OUTPUT:
%       waypointMap  list of [x,y] coords in order from pose to goal


% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu





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
            k = k+1;
        end
    end
end


%initiate obs
obs(1).x = [];
obs(1).y = [];
obs(1).length=0;

%for all obstacles
for i=1:I
    
    index = find(nodeMap(:,1)==i);
        %initiate next cell
        obs(i).x = [];
        obs(i).y = [];
        
    %for all obstacles
    for j=index(1):index(end) -1
        

        %decompose head / tail
        head = nodeMap(j,2:3);
        tail = nodeMap(j+1,2:3);
        
        %calc normVector of head to tail
        vect = tail-head;
        dist = norm(vect);
        normVect = vect ./ dist;
        
        %create rectangle around point.
        a = head - normVect .* robRad - flip(normVect) * robRad;
        b = head - normVect .* robRad + flip(normVect) * robRad;
        c = tail + normVect .* robRad + flip(normVect) * robRad;
        d = tail + normVect .* robRad - flip(normVect) * robRad;
        
        
        %create objects
    obs(i).x = [obs(i).x , [a(1),b(1),c(1),d(1)]];
    obs(i).y = [obs(i).y , [a(2),b(2),c(2),d(2)]];
    obs(i).length = length(obs(i).x);

    end %end for j

end %end for i
    
maxLen = 0;
for i=1:I
  if maxLen < obs(i).length
      maxLen = obs(i).length;
  end %end if
end %end find max loop if

newMap = zeros(I,J*2);

%for each possible obstacle
for i=1:I
    hull_index = [];

    hull_index = convhull(obs(i).x,obs(i).y);


    %for each x in hull_index -1
    for j=1:2:length(hull_index).*2 -2
        %                                                                  
        newMap(i,j:j+1) = [obs(i).x(hull_index(ceil(j/2))),obs(i).y(hull_index(ceil(j/2)))];
    end %end for j

end %for newman for statement


%plotMap('hw7b.txt',[mapsize(1),mapsize(3),mapsize(2),mapsize(4)]);
     



[waypointMap] = buildRRTpoint(newMap,mapsize,pose,goal,delta);
end %end function
