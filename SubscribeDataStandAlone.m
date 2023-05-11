function [bsSampFrame, imgHeader, imgL,imgR, imuSampFrame, golden, depth,depthR] = SubscribeDataStandAlone(combinedata)

            temp = str2num(combinedata.strings{1});
            imgHeader(1) = temp(1);
            imgHeader(2) = temp(2)*1000 + temp(3)/1000000;
            imgHeader(3) = temp(4);
            imgHeader(4) = temp(5);
            
            imgLbuffer = matlab.net.base64decode(combinedata.strings{2});
            imgL = reshape(imgLbuffer,3,imgHeader(4),imgHeader(3));
            imgL = uint8(permute(imgL,[3 2 1]));
            
            imgRbuffer = matlab.net.base64decode(combinedata.strings{3});
            imgR = reshape(imgRbuffer,3,imgHeader(4),imgHeader(3));
            imgR = uint8(permute(imgR,[3 2 1]));
            
            tempDir = pwd;
            depthBuffer = matlab.net.base64decode(combinedata.strings{4});
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
            depthR = depth;
            odom = reshape(str2num(combinedata.strings{5}),12,[])';
            bsSampFrame = [odom(:,1),odom(:,2)*1000+ (odom(:,3)/1000000),odom(:,2)*1000+ (odom(:,3)/1000000),odom(:,4),odom(:,5),zeros(Rows(odom),1)];
            golden = odom(:,6:12);
            
            temp = reshape(str2num(combinedata.strings{6}),13,[])';
            imuSampFrame = [temp(:,1), temp(:,2)*1000+ (temp(:,3)/1000000), temp(:,4:13)];
        end