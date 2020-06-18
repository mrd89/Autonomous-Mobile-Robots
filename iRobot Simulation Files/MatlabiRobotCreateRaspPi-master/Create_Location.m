function [P,varargout] = Create_Location(N,pbool)
%Create_Location Retrieve Create robot pose from overhead localization
% system.
%
%  P = Create_Location(N) returns
%    * N should be a row vector consisting of the Id's of the robots for wich 
%      you wish to get the location.
%    * The size(N) row by 5 matrix P for a vector N with length(N)as well
%      as printing the values of Timestamp, Id, X, Y, and Theta on the
%      screen for each value of N.
%    * pbool is a boolean that defines whether Create_Location should print
%      to screen
%    * P is of the form [ID X Y Theta Timestamp] where each: ID, X, Y,
%      Theta, and Timestamp is a length(N) column vector
%
%  [ID,X,Y,Theta,TS] = Create_Location(N) returns
%    * The list of retrieved id's ID
%    * The x position of each robot retrieved X
%    * The y position of each robot retrieved Y
%    * The orientation of each robot retrieved Theta
%    * The time the location was sent TS
%
%  NOTE:
%    * Output is put in numerical order according to Id number
%    * If robot is not seen in the field then values of zero are
%      substituted for real data.  This can be checked by making sure
%      Timestamp is not zero.  If a robot was detected Timestamp for that
%      robot will not be zero.
%    * A Warning Mesage stating "Warning: The specified amount of data was
%      not returned within the Timeout period." will appear at the
%      beginning of the output.  This is perfectly normal and will not
%      affect the functionality of the program.  

%   Author(s): A. O'Connell, A. Beg, C. Yang, H. Nien
%   $Date: 2010/03/18  9:19pm$
%
%   Further modified by C. Yang to work with newer version of overhead 
% localization system.
%   $Date: 2011/01/19

if nargin==0
    error('Overhead:NoIds','Too few arguments.  See help Create_Location'); 
elseif nargin == 1
    pbool = false;
elseif nargin>2
    error('Overhead:TooManyArguments','Too many arguments.  See help Create_Location');
end

%remote host (character string ip address with .'s)
rh='10.253.194.13';
%remote port (integer value)
% port=8866; %old port value
port=60066;

%get UDP packet from the remote host
u=udp(rh,port,'LocalPort',port);

%open the packet and
fopen(u);
onCleanup(@()fclose(u));
%count = length(N)*19;

%read in packet and get size
[packet count] = fread(u);

%if statement used to prevent breakdown when no robots detected in the
%field
msg_struct_len = 21;
if count>=msg_struct_len+1
    
    %parse object into useable data
    packet_size = packet(1,1);

    %initialize parsing variables
    Id(1:(count-1)/msg_struct_len)=0;
    X(1:(count-1)/msg_struct_len)=0;
    Y(1:(count-1)/msg_struct_len)=0;
    Theta(1:(count-1)/msg_struct_len)=0;
    Timestamp(1:(count-1)/msg_struct_len)=0;
    %iterate for multiple robots
    for i=1:1:(count-1)/msg_struct_len
        Id(i) = packet((i-1)*msg_struct_len+2,1);
        X(i) = typecast(uint8(packet((i-1)*msg_struct_len+3:(i-1)*msg_struct_len+6,1)),'single');
        Y(i) = typecast(uint8(packet((i-1)*msg_struct_len+7:(i-1)*msg_struct_len+10,1)),'single');
        Theta(i) = typecast(uint8(packet((i-1)*msg_struct_len+11:(i-1)*msg_struct_len+14,1)),'single');
        Timestamp(i) = typecast(uint8(packet((i-1)*msg_struct_len+15:(i-1)*msg_struct_len+22,1)),'double');
%         Timestamp(i) = (packet((i-1)*msg_struct_len+4,1)*2^32+packet(5,1)*2^16+packet(6,1)*2^8+packet(7,1)*2^0)/1000;
    end
    
%     %reorder data in asending order
%     N=sort(N);
    % Create matrix of the ID(col.1),Pose(2:4), and Timestamp(5)
    L(1:length(N),1:5)=0;
    for i=1:length(N)
        L(i,1)=N(i);
        for k=1:size(Id,2)
            %Search for desired Id
            if Id(k)==N(i)
                L(i,2)=X(k);
                L(i,3)=Y(k);
                L(i,4)=Theta(k);
                L(i,5)=Timestamp(k);
                
                %quit searching when Id is found
                break
                
            end
        end
        %Notify user if a desired Id is not within the packet
        if L(i,5)==0
            fprintf(1,'\nCannot see ID %d\n',N(i));
        end
    end
else
    L(1:length(N),1:5)=0;
    fprintf(1,'\nThere is no robot on the field!\n');
end

%Notify user of the Index corresponding to each Id
if(pbool)
    fprintf(1,'\nID to Index Conversion:\n ID  Index\n')
    for i=1:length(N)
        fprintf(1,'  %d   %d\n',N(i),i)
    end
end

%Organize output
no=nargout;
%ie Create_Location(N)=>notify user that they did not provide an output
if no==0
    for i=1:length(N)
        fprintf(1,'Timestamp: %0.3f, ID: %d, X: %0.3f, Y: %0.3f, Theta: %0.3f',L(i,5), L(i,1), L(i,2), L(i,3), L(i,4))
    end
    fprintf(1,'\nNo data output to workspace \nPlease provide output variable if you want to use location information \n(see: help Create_Location)\n')
    %ie P=Create_Location(N)
elseif no==1
    P=L;
else
    P=L(:,1);
    varargout=num2cell(L(:,2:no),1);
end

% fclose(u)
delete(u)
clear u