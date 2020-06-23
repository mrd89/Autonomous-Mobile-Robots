function objHdl = plotEdge(newNode,oldNode,figHdl)
% Plots a pair of connected nodes in an existing figure window.
% 
% INPUTS:
%       newNode     1-by-2 vector defining the x-y position if the new node
%       oldNode     1-by-2 vector defining the x-y position if the old node
%
% OPTIONAL INPUTS:
%       figHdl      Figure handle to add node (default: current figure)
% 
% OUTPUTS:
%       objHdl      2-dimensional vector of handles of objects plotted
%

if ~exist('figHdl','var') || isempty(figHdl), figHdl = gcf; end

figure(figHdl)
hold on
objHdl(1) = plot(newNode(1),newNode(2),'r*');

if ~isempty(oldNode)
    objHdl(2) = plot(oldNode(1),oldNode(2),'b*');
    objHdl(3) = plot([oldNode(1) newNode(1)],[oldNode(2) newNode(2)],'r');
end
