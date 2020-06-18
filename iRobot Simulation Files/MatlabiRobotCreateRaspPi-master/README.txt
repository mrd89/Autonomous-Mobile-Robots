This toolbox is based on Esposito's Matlab Toolbox for the iRobot Create with modifications shown in the change log.
http://www.usna.edu/Users/weapsys/esposito/roomba.matlab/

Change Log:
CreateBeagleInit.m instead of RoombaInit.m for Wifi and beagleboard.
ReadSonar.m to read 3 sonars measurement.
ReadBeacon.m to read ARtag's measurement from the camera.
Make following function/command send only one fwrite:
	* AngleSensorRoomba.m
	* BatteryChargeReaderRoomba.m
	* BatteryVoltageRoomba.m
	* BumpWheelDropsSensorsRoomba.m
	* ButtonsSensorRoomba.m
	* CliffFrontLeftSensorRoomba.m
	* CliffFrontLeftSignalStrengthRoomba.m
	* CliffFrontRightSensorRoomba.m
	* CliffFrontLeftSignalStrengthRoomba.m
	* CliffFrontRightSensorRoomba.m
	* CliffFrontRightSignalStrengthRoomba.m
	* CliffLeftSensorRoomba.m
	* CliffLeftSignalStrengthRoomba.m
	* CliffRightSensorRoomba.m
	* CliffRightSignalStrengthRoomba.m
	* CurrentTesterRoomba.m
	* DemoCmdsCreate.m
	* DistanceSensorRoomba.m
	* LEDsRoomba.m
	* SetDriveWheeelsCreate.m
	* SetFwdVelAngVelCreate.m
	* SetFwdVelRadiusRoomba.m
	* SetLEDsRoomba.m
	* TestSonar.m
	* travelDist.m
	* turnAngle.m
Added CalibGUI.m for beacon and sonar calibration.
Added beagleboard control functions: BeagleControl.m and BeagleShutdown.m.
Added PlotCreateVideo.m for stream video from camera.
Added judp.m for udp connection with bigger packets for streaming video.