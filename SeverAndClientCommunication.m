classdef SeverAndClientCommunication
    properties
        gazebo;
        cmdVelPub;
        combine_data;
    end
    
    methods
        function obj = SeverAndClientCommunication(serverIP, clientIP)
            setenv('ROS_MASTER_URI',serverIP);
            setenv('ROS_IP',clientIP);
            rosinit;
            obj.gazebo = ExampleHelperGazeboCommunicator;
            obj.cmdVelPub = rospublisher('/cmd_vel',rostype.geometry_msgs_Twist);
            pause(1) % Wait to ensure publisher is setup
            obj.combine_data = rossubscriber('/combinationInfo', 'BufferSize', 30);
            disp('sever is connected!')
            SetCmdVelocity(obj, 0, 0)
            
            ResumeSystem(obj)
%             pause(1) % Wait to ensure publisher is setup

            disp('Initialization is done!')
        end
        
        function delete(obj)
            rosshutdown;
            disp('sever is disconnected!')
        end
        
        function ResumeSystem(obj)
            resumeSim(obj.gazebo);
            disp('System is resumed.')
        end
        
        function SetCmdVelocity(obj, v, w)
            cmdMsg = rosmessage(obj.cmdVelPub);
            cmdMsg.Linear.X = v;
            cmdMsg.Angular.Z = w;
            send(obj.cmdVelPub,cmdMsg);
        end
        
        function [bsSampFrame, imgHeader, imgL,imgR, imuSampFrame, golden, depth] = SubscribeData(obj)
            resumeSim(obj.gazebo)
            try
                combinedata = receive(obj.combine_data, 10);
            catch
                warning('data loss')
                for fail_count = 1:5
                    try
                        ResumeSystem(obj)
                        combinedata = receive(obj.combine_data, 50);
                        warning('data loss number:')
                        disp(fail_count)
                        break;
                    catch
                        if fail_count == 5
                            assert(fail,'data loss number > 5')
                        end
                    end
                end
            end
            temp = str2num(combinedata.Strings{1});
            imgHeader(1) = temp(1);
            imgHeader(2) = temp(2)*1000 + temp(3)/1000000;
            imgHeader(3) = temp(4);
            imgHeader(4) = temp(5);
            
            imgLbuffer = matlab.net.base64decode(combinedata.Strings{2});
            imgL = reshape(imgLbuffer,3,imgHeader(4),imgHeader(3));
            imgL = uint8(permute(imgL,[3 2 1]));
            
            imgRbuffer = matlab.net.base64decode(combinedata.Strings{3});
            imgR = reshape(imgRbuffer,3,imgHeader(4),imgHeader(3));
            imgR = uint8(permute(imgR,[3 2 1]));
            
            tempDir = pwd;
            depthBuffer = matlab.net.base64decode(combinedata.Strings{4});
            calibRomFid = fopen(fullfile(tempDir,'data1.bin'),'w');
            fwrite(calibRomFid, depthBuffer,'uint8');
            fclose(calibRomFid);
            fid=fopen(fullfile(tempDir,'data1.bin'),'rb');
            AA = fread(fid,'single');
            fclose(fid);
            
            try
                depth = 1000.*reshape(AA,imgHeader(4),imgHeader(3))';
            catch
                depth = imgL(:,:,1);
            end
            
            odom = reshape(str2num(combinedata.Strings{5}),12,[])';
            bsSampFrame = [odom(:,1),odom(:,2)*1000+ (odom(:,3)/1000000),odom(:,2)*1000+ (odom(:,3)/1000000),odom(:,4),odom(:,5),zeros(Rows(odom),1)];
            golden = odom(:,6:12);
            
            temp = reshape(str2num(combinedata.Strings{6}),13,[])';
            imuSampFrame = [temp(:,1), temp(:,2)*1000+ (temp(:,3)/1000000), temp(:,4:13)];
        end
    end
end