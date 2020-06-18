function travelDist(serPort, roombaSpeed, distance);
%travelDist(serPort, roombaSpeed, distance)
%Moves the Create the distance entered in meters. Positive distances move the
%Create foward, negative distances move the Create backwards.
%roombaSpeed should be between 0.025 and 0.5 m/s
% By; Joel Esposito, US Naval Academy, 2011
% Modified by: Chuck Yang, ty244, 2012

try
    
    %Flush Buffer
    N = serPort.BytesAvailable();
    while(N~=0)
        fread(serPort,N);
        N = serPort.BytesAvailable();
    end
    
    warning off
    global td
    if (roombaSpeed < 0) %Speed given by user shouldn't be negative
        disp('WARNING: Speed inputted is negative. Should be positive. Taking the absolute value');
        roombaSpeed = abs(roombaSpeed);
    end
    
    if (abs(roombaSpeed) < .025) %Speed too low
        disp('WARNING: Speed inputted is too low. Setting speed to minimum, .025 m/s');
        roombaSpeed = .025;
    end
    
    if (distance < 0) %Definition of SetFwdVelRAdius Roomba, speed has to be negative to go backwards. Takes care of this case. User shouldn't worry about negative speeds
        roombaSpeed = -1 * roombaSpeed;
    end
    
    [computerType, maxSize, endian] = computer;
    isLittleEndian = (endian == 'L');
    
    if (distance ~=0)
        FwdVelMM = min( max(roombaSpeed,-.5), .5)*1000;
        RadiusMM = 32768;
        distanceMM = distance * 1000;
        if (isLittleEndian)
            FwdVelMM = typecast(swapbytes(int16(FwdVelMM)),'uint8');
            RadiusMM = typecast(swapbytes(int16(RadiusMM)),'uint8');
            distanceMM = typecast(swapbytes(int16(distanceMM)),'uint8');
        else
            FwdVelMM = typecast(int16(FwdVelMM),'uint8');
            RadiusMM = typecast(int16(RadiusMM),'uint8');
            distanceMM = typecast(int16(distanceMM),'uint8');
        end
        fwrite(serPort, [137 FwdVelMM RadiusMM 156 distanceMM 137 0 0 0 0 154]);
        pause(td)
        while( serPort.BytesAvailable() ==0)
            %disp('waiting to finish')
        end
        %disp('Done travelDist.')
        pause(td)
        
    end
catch
            disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
    end