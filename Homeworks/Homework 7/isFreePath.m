function [hits,dist] = isFreePath(obstacles,head,tail,samples)
% This function finds if the tail is visible to the head with a map with
% obstacles formed by polygons
%
%
% INPUT:
%       obstacles    I-by-J map of obstacles
%       head        [x,y] Location of start node
%       tail        [x,y] Location of end node
%       samples     number of samples on the vector from h2t

% OUTPUT:
%       hits        number of hits with obstacles (exclude tail/head)
%       dist        distance from head to tail


% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu

I = max(obstacles(:,1));

%vector from Node i to Node j
vect = (tail-head);
dist = norm(vect);
normvect = vect ./ dist;

%create vector of points along line of
x = head(1) + normvect(1) .* linspace(0,dist,samples);
y = head(2) + normvect(2) .* linspace(0,dist,samples);



% for all obstacles
in=0;
for k = 1:I
    
    index = find(obstacles(:,1)==k);
    [in] = in+inpolygon(x',y',obstacles([index],2),obstacles([index],3));
    
end %end for I

hits = sum(in(2:end-1));
end %end function
