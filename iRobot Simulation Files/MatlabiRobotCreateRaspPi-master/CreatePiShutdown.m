function CreatePiShutdown(Ports)
%
%   The object 'serPort' must first be initialized with the 
%   CreateBeagleInit command (available as part of the Matlab Toolbox for 
%   the iRobot Create).
%
% By: Liran 1/2019

% Before closing communication stop the robot in case it is moving
SetFwdVelAngVelCreate(Ports.create, 0,0);
pause(1);

% Send stop command to terminate the loop on the Pi
data_to_send = ('stop');
fwrite(Ports.create, data_to_send);
pause(1);
 
 
 % Clean up
try
    
    if (strcmp(Ports.create.status,'open'))
        fclose(Ports.create);
        pause(0.1);
    end
    	
	if (strcmp(Ports.dist.status,'open'))
		fclose(Ports.dist);
        pause(0.1);
    end
	
	if (strcmp(Ports.tag.status,'open'))
		fclose(Ports.tag);
		pause(0.1);
    end	
    
    delete(Ports.create);
    delete(Ports.dist);
	delete(Ports.tag);
    
catch
    disp('WARNING:  Function did not terminate correctly.  Output may be unreliable.')
end

end