function RealSenseDepthImage(serPort, height);
%RealSenseDepthImage(serPort) displays in a figure the current Depth image seen by
%the RealSense camera. The output figure has a white horizontal line
%indicating the region of the image as specified by the [height] inputted
%as angle in degrees.
%Requires: 1<=[height]<=39
    
%Flush Buffer    
flushinput(serPort.cmd);

warning off
global td
data_to_send = uint8('depth');
fwrite(serPort.cmd, data_to_send);

disp('waiting for response');
while serPort.cmd.BytesAvailable==0
    pause(3);
end

% Get response 
[resp read_count] = fread(serPort.cmd, serPort.cmd.BytesAvailable, 'uint16'); 

if resp == 99
    disp('No camera connected, cannot call this function')
else
	if read_count<307200
		disp('incomplete frame received, the bottom of image might be trimmed');
	end

	im_height = floor(read_count/640);
	img = reshape(resp(1:640*im_height(1)), 640, im_height(1));
	read_count
	size(img)

	% now prepare to draw a  line at the specified height in image
	pix_per_deg = 480/40;
	for i=1:640
		for j=(40-height)*pix_per_deg-2:(40-height)*pix_per_deg
			img(i, j) = 65535;
		end
	end

	img = flip(img, 2);
	img = imrotate(img, 90);
	figure;
	%colormap('gray');
	imagesc(img, [0 65535]);
	colorbar;
end

flushinput(serPort.cmd);
pause(td)

end