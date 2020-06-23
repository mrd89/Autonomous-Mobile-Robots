function [minWeight,minPath] = findPath(nodeMap,roadMap,pose,goal)
% This function takes in a map and ouputs the roadmap covering Q_Free
%
%
%
% INPUT:
%       nodeMap     [o,x,y], N-BY-3 of each node where 0 is obstacle number
%       roadMap     Output of potential field N by N
%       pose        [x,y] current position
%       goal        [x,y] goal position


% OUTPUT:
%       minWeight   minimum distance 
%       minPath     shortest path in terms of the nodes


% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu


N = length(nodeMap(:,1));

minGoal = intmax;
goal_index = NaN;
minPose = intmax;
pose_index = NaN;

%find minimum unabstructed path to roadmap


minWeight = intmax;
minPath = NaN;
for i =1:N
    [posHits,posDist]= isFreePath(nodeMap,pose,nodeMap(i,2:3),40);
    if posHits<1
        for j=1:N
            [goalHits,goalDist]= isFreePath(nodeMap,goal,nodeMap(j,2:3),40);
            if goalHits<1
                [weight,path] =dijkstra(roadMap,i,j);
                weight = weight + posDist + goalDist;
                
                if weight <minWeight
                    minWeight = weight;
                    minPath = path;
                end
                
            end
        end
    end
end



end %end function




