function [nodeMap,roadMap] = createRoadmap(map)
% This function takes in a map and ouputs the roadmap covering Q_Free
%
%
%
% INPUT:
%       map         polygon map input


% OUTPUT:
%       nodeMap    [o,x,y], N-BY-3 of each node where 0 is obstacle number
%       roadMap    Output of potential field N by N

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu


%find lenghth of map
I = length(map(:,1)); %also number of obstacles
J = length(map(1,:));

L = I.*J /2;
N = L- sum(sum(map==0))/2;

%initiate nodes matrix
nodeMap = zeros(N,3);

%initiate roadMap
roadMap = zeros(N,N);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% decompose obstacles into matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = 1 ;
for i=1:I %for obstacles
    for j=1:2:J %for nodes in obstacle
        
        %decompose x,y
        x = map(i,j);
        y = map(i,j+1);
        
        %if both nonzero
        if ~( x ==0 && y ==0)
            %give nodemap corresponding values
            nodeMap(k,:) = [ i,x,y];
            k = k+1;%kep track of node
        end
    end
end



for i =1:N %for all nodes
    
    
    head = nodeMap(i,2:3);
    
    %check all nodes except self
    for j = 1:N
        
        %no need to check self
        if ~(i ==j)
            
            %end point here
            tail = nodeMap(j,2:3);
            
            %function checks if the path is free
            [hits,dist]= isFreePath(nodeMap,head,tail,40);
            
            
            %if one of the samples hit a obstacle
            if hits<1
                
                %set map as distance between nodes
                roadMap(i,j) = dist;
                
                %if both in same node
            elseif(nodeMap(i,1) == nodeMap(j,1)) %check for case of adjacent nodes
                
                currNode = nodeMap(i,1);
                if (abs(i-j)==1 || abs(i-j) == length(find(nodeMap(:,1)==currNode))-1)
                    %set map as distance between nodes
                    roadMap(i,j) = dist;
                end
            end
        end %end if i/= j

    end %end j
end %end i
end %end function