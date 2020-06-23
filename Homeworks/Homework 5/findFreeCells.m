function[freeCells] = findFreeCells(head,tail)
%this funcion takes in the head and tail of the range sensor and is able to
%output the cells inbetween these (including the head and tail)\

%This function is based on Bresenham's line drawing algorithm rewritten for 
%this specific application. 
% https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
%
%       INPUTS:
%           Head        [i,j] index of head of range sensor
%           tail        [i,j] index of tail of range sensor
%
%       OUTPUTS:
%           freeCells   Vector [nx2] of free cells (inclusive head&tail)

% solve for abs of dx,dy
dx=abs(tail(1) - head(1));
dy=abs(tail(2) - head(2));

%calculate the slope
m=abs(dy)>abs(dx);

%if the slope is greater than 1
if m
    %here we swap dx and dy
    temp=dx;
    dx=dy;
    dy=temp;
end

%if there is no difference in y direction
if dy==0
    %create dx+1 by 1 vector
    vect=zeros(dx+1,1);
else %else there is a dy
    vect=[0;diff(mod( [ floor(dx / 2): -dy : -dy * dx+floor(dx/2) ]', dx ))>=0];
end


%if dy greater than dx
if m
    %if the head y <= tail y
    if (head(2)<=tail(2))
        % y gets the vector of head(y) to tail(y)
        y=[head(2):tail(2)]';
        
    else
        %else Y gets head to tail with step -1
        y=[head(2):-1:tail(2)]';
    end
    
    if (head(1)<=tail(1))
        x=head(1)+cumsum(vect);
    else
        x=head(1)-cumsum(vect);
    end
else %else dx greater than dx
    %Complete same calc on x (as above)
    if (head(1)<=tail(1))
        
        % x gets the vector of head(x) to tail(x)
        x=[head(1):tail(1)]';
    else
        
        %else Y gets head to tail with step -1
        x=[head(1):-1:tail(1)]';
    end
    if (head(2)<=tail(2))
        
        y=head(2)+cumsum(vect);
    else
        
        y=head(2)-cumsum(vect);
    end
end

%set output to single vector
freeCells = [x,y]';
