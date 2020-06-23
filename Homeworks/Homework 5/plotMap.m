function tt =plotMap(map)
%this function plots a map of size [nx4]
for i=1:length(map(:,1))
    tt =plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'r');
end
end

