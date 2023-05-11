classdef CamImuBinFileReader < CameraInputInterface
    
    properties (Constant)
        CAMIMU_HDR_MAGIC = hex2dec('20170710');
        
        CAMFRAMEINFO_CNT = 1;
        CAMFRAMEINFO_ID  = 2;
        CAMFRAMEINFO_TIMESTAMP = 3;
        CAMFRAMEINFO_SYSTIMESTAMP = 4;
        CAMFRAMEINFO_IMAGEOFFSET = 5;
        
        IMUSAMP_ID = 1;
        IMUSAMP_TIMESTAMP = 2;
        IMUSAMP_SYSTIMESTAMP = 3;
        IMUSAMP_GYRO_DATA = [5,4,6];
        IMUSAMP_ACCL_DATA = [8,7,9];
        IMUSAMP_FRAMECNT = 10;
        
        COLOR_IMU_GYRO_X   = [1,0,0]  % r
        COLOR_IMU_GYRO_Y   = [1,1,0]  % y
        COLOR_IMU_GYRO_Z   = [1,0,1]  % m
        COLOR_IMU_ACCL_X   = [0,1,1]  % c
        COLOR_IMU_ACCL_Y   = [0,0,0]  % k
        COLOR_IMU_ACCL_Z   = [0.2081, 0.1663, 0.5292]
    end
    
    properties
        camImuBinFid
        camFrameFields
        imuSampleFields
        imuSampList
        vidFrameInfoList
        imgSize  % [height, width, channels]
        
        currFrameInd;
    end
    
    methods
        function obj = CamImuBinFileReader()
            obj.camImuBinFid = [];
            obj.imuSampList = [];
            obj.vidFrameInfoList = [];
            obj.imgSize = [];
            obj.camFrameFields = 3;        % ID (uint32), timestamp (uint32), systimestamp (uint32)
            obj.imuSampleFields = 9;       % ID (uint32), timestamp (uint32), systimestamp (uint32), gyro[3] (float), acc[3] (float)
            
            obj.currFrameInd = 0;
        end
        
        function Open(obj, camImuBinFilePath)
            global IMU_BODYSNESOR_RAW_DATA_FIG
            
            ReadFrameTimeStampAndImuData(obj, camImuBinFilePath);
            fprintf('Total %d frames, %d IMU samples are read are read from %s\n', Rows(obj.vidFrameInfoList), Rows(obj.imuSampList), camImuBinFilePath);
            
            obj.currFrameInd = 1;
            
            if ~isempty(IMU_BODYSNESOR_RAW_DATA_FIG)
                figure(IMU_BODYSNESOR_RAW_DATA_FIG),subplot(2,1,1), PlotSamples(obj, 'ImuAccl'), hold on;
                figure(IMU_BODYSNESOR_RAW_DATA_FIG),subplot(2,1,2), PlotSamples(obj, 'ImuGyro'), hold on;
            end
        end
        
        
        function Close(obj)
            if (~isempty(obj.camImuBinFid) && obj.camImuBinFid > 0)
                fclose(obj.camImuBinFid);
                obj.camImuBinFid = [];
            end
            
            obj.imuSampList = [];
            obj.vidFrameInfoList = [];
            obj.imgSize = [];
            
            obj.currFrameInd = 0;
        end
        
        function ReadFrameTimeStampAndImuData(obj, camImuBinFilePath)
            assert(isempty(obj.camImuBinFid), 'A camImuBinFile are in opening state.');
            [obj.camImuBinFid, errMsg] = fopen(camImuBinFilePath);
            if (obj.camImuBinFid <= 0)
                error('Cannot open %s for read: %s', camImuBinFilePath, errMsg);
            end
            
            frameCnt = 0;  % 0 based
            vidImgDataOffsetBytes = 0;
            while ~feof(obj.camImuBinFid)
                % Read header
                magicNum = fread(obj.camImuBinFid, 1, 'uint32');
                if isempty(magicNum)
                    assert(feof(obj.camImuBinFid) ~= 0, 'Incomplete camImuBinFile at frame %d', frameCnt);
                    break;
                end
                assert(magicNum == obj.CAMIMU_HDR_MAGIC, 'Doesnot match magic number at frame %d', frameCnt);
                numImuSamp = fread(obj.camImuBinFid, 1, 'uint16');
                assert(~isempty(numImuSamp), 'Incomplete camImuBinFile at frame %d', frameCnt);
                imgWidth = fread(obj.camImuBinFid, 1, 'uint16');
                assert(~isempty(imgWidth), 'Incomplete camImuBinFile at frame %d', frameCnt);
                imgHeight = fread(obj.camImuBinFid, 1, 'uint16');
                assert(~isempty(imgHeight), 'Incomplete camImuBinFile at frame %d', frameCnt);
                numImgChnl = fread(obj.camImuBinFid, 1, 'uint16');
                assert(~isempty(numImgChnl), 'Incomplete camImuBinFile at frame %d', frameCnt);
                if isempty(obj.imgSize)
                    obj.imgSize = [imgHeight, imgWidth, numImgChnl];
                elseif any(obj.imgSize ~= [imgHeight, imgWidth, numImgChnl])
                    error('Image size change detected at frame %d: h=%d, w=%d, c=%d', frameCnt, imgHeight, imgWidth, numImgChnl);
                end
                payloadSize = fread(obj.camImuBinFid, 1, 'uint32');
                assert(~isempty(payloadSize), 'Incomplete camImuBinFile at frame %d', frameCnt);
                
                vidImgDataOffsetBytes = vidImgDataOffsetBytes + 16;  % 16 byte header
                
                % Read IMU samples
                % Format of an IMU sample:
                % uint32 id
                % uint32 timeStamp
                % uint32 systimestamp
                % float gyroData[3]
                % float accData[3]
                [imuFrameSampList, rdCnt] = fread(obj.camImuBinFid, numImuSamp*obj.imuSampleFields, 'single');
                assert(rdCnt == numImuSamp*obj.imuSampleFields, 'Incomplete camImuBinFile at frame %d', frameCnt);
                imuFrameSampList = [reshape(imuFrameSampList, obj.imuSampleFields, [])', frameCnt(ones(numImuSamp, 1))];
                intFldIdx = [obj.IMUSAMP_ID, obj.IMUSAMP_TIMESTAMP, obj.IMUSAMP_SYSTIMESTAMP];
                imuFrameSampList(:, intFldIdx) = reshape(typecast(Vec(single(imuFrameSampList(:, intFldIdx))), 'uint32'), [], length(intFldIdx));  % ID, timestamp, systimestamp
                obj.imuSampList = [obj.imuSampList; imuFrameSampList];
                vidImgDataOffsetBytes = vidImgDataOffsetBytes + 4 * obj.imuSampleFields * numImuSamp; % imu data in bytes
                
                
                % [frameCnt, vidFrameTimeStamp, vidImageOffset]
                % video frame id
                vidFrameId = fread(obj.camImuBinFid, 1, 'uint32');
                assert(~isempty(vidFrameId), 'Incomplete camImuBinFile at frame %d', frameCnt);
                % video frame timestamp
                vidFrameTimeStamp = fread(obj.camImuBinFid, 1, 'uint32');
                assert(~isempty(vidFrameTimeStamp), 'Incomplete camImuBinFile at frame %d', frameCnt);
                % video frame systimestamp
                vidFrameSysTimeStamp = fread(obj.camImuBinFid, 1, 'uint32');
                assert(~isempty(vidFrameSysTimeStamp), 'Incomplete camImuBinFile at frame %d', frameCnt);
                
                vidImgDataOffsetBytes = vidImgDataOffsetBytes + 4*obj.camFrameFields; % video frame data offset in bytes
                vidFrameInfo = [frameCnt, vidFrameId, vidFrameTimeStamp, vidFrameSysTimeStamp, vidImgDataOffsetBytes];
                obj.vidFrameInfoList = [obj.vidFrameInfoList; vidFrameInfo];
                
                imgBytes = imgWidth * imgHeight * numImgChnl;
                vidImgDataOffsetBytes = vidImgDataOffsetBytes + imgBytes;
                
                status = fseek(obj.camImuBinFid, imgBytes, 'cof');                

%                 assert(status == 0, 'fseek error at frame %d', frameCnt);
                if status == -1
                    warning('fseek error at frame %d', frameCnt)
                    break;
                end
                
                frameCnt = frameCnt + 1;
            end
            
            % Rewind camImuBinFile
            fseek(obj.camImuBinFid, 0, 'bof');
        end
        
        function varargout = GetFrameImage(obj, frameId)
            % frameId is 0 based
            assert(frameId < Rows(obj.vidFrameInfoList), 'frameId is out of range: 0 - %d', Rows(obj.vidFrameInfoList) - 1);
            fseek(obj.camImuBinFid, obj.vidFrameInfoList(frameId + 1, obj.CAMFRAMEINFO_IMAGEOFFSET), 'bof');
            stereoImage = uint8(fread(obj.camImuBinFid, prod(obj.imgSize), 'uint8'));
            if (obj.imgSize(3) == 3)
                stereoImage = permute(reshape(stereoImage, [obj.imgSize(3), obj.imgSize(2), obj.imgSize(1)]), [3,2,1]);
            elseif (obj.imgSize(3) == 1)
                stereoImage = reshape(stereoImage, [obj.imgSize(2), obj.imgSize(1)])';
            else
                assert(false);
            end
            
            if (nargout == 1)
                varargout{1} = stereoImage;
            elseif (nargout == 2)
                varargout{1} = stereoImage(:, 1:obj.imgSize(2)/2, :);
                varargout{2} = stereoImage(:, obj.imgSize(2)/2:end, :);
            else
                error('Too many output arguments');
            end
        end
       
        function AlignBodySensorToCamera(obj, timeOffset)
            if ~exist('timeOffset', 'var')
                timeOffset = 0;
            end
            
            numFrames = GetNumFrames(obj);
            numBodySensorSamp = Rows(obj.bodySensorSampList);
            
            obj.bodySensorSampList = [obj.bodySensorSampList, zeros(Rows(obj.bodySensorSampList), 1)];
            frameStartSampInd = 1;
            for iFrm = 1:numFrames
                frameCnt = obj.vidFrameInfoList(iFrm, obj.CAMFRAMEINFO_CNT);
                curFrmTimeStamp = obj.vidFrameInfoList(iFrm, obj.CAMFRAMEINFO_SYSTIMESTAMP);
                endTimeStamp = obj.bodySensorSampList(find(obj.bodySensorSampList(:,obj.BODYSENSORSAMP_SYSTIMESTAMP) + timeOffset >= curFrmTimeStamp, 1, 'first'), obj.BODYSENSORSAMP_SYSTIMESTAMP);
                if isempty(endTimeStamp)
                    break;
                end
                frameEndSampInd = find(obj.bodySensorSampList(:,obj.BODYSENSORSAMP_SYSTIMESTAMP) == endTimeStamp, 1, 'last');
                obj.bodySensorSampList(frameStartSampInd : frameEndSampInd, obj.BODYSENSORSAMP_FRAMECNT) = frameCnt;
                frameStartSampInd = frameEndSampInd + 1;
                if (frameStartSampInd > numBodySensorSamp)
                    break;
                end
            end
        end
        
        function numFrames = GetNumFrames(obj)
            numFrames = Rows(obj.vidFrameInfoList);
        end
        
        function [frame, timeStamp, frameInd] = GetNextFrame(obj)
            frame = GetFrameImage(obj, obj.currFrameInd);
            frameInd = obj.currFrameInd;
            timeStamp = obj.vidFrameInfoList(frameInd, obj.CAMFRAMEINFO_SYSTIMESTAMP);
            if HasFrame(obj)
                obj.currFrameInd = obj.currFrameInd + 1;
            end
        end
        
        function b = HasFrame(obj)
            b = obj.currFrameInd <= GetNumFrames(obj);
        end
        
        function PlotSamples(obj, varargin)
%            allowedPlot = {'BodySensorV', 'BodySensorW', 'ImuGyroX', 'ImuGyroY', 'ImuGyroZ', 'ImuAccX', 'ImuAccY', 'ImuAccZ'};
            whatToPlot = unique(varargin);
            
            if ~isempty(obj.imuSampList)
                imuTimeStamp = obj.imuSampList(:, obj.IMUSAMP_SYSTIMESTAMP);
                imuGyroX = obj.imuSampList(:,obj.IMUSAMP_GYRO_DATA(1));
                imuGyroY = obj.imuSampList(:,obj.IMUSAMP_GYRO_DATA(2));
                imuGyroZ = obj.imuSampList(:,obj.IMUSAMP_GYRO_DATA(3));
                imuAcclX = obj.imuSampList(:,obj.IMUSAMP_ACCL_DATA(1));
                imuAcclY = obj.imuSampList(:,obj.IMUSAMP_ACCL_DATA(2));
                imuAcclZ = obj.imuSampList(:,obj.IMUSAMP_ACCL_DATA(3));
            end
            
            plotColor = [];
            plotLegend = [];
            plotArgs = [];

            if any(strcmp(whatToPlot, 'ImuGyro'))
                assert(~isempty(obj.imuSampList), 'No imu data');
                plotArgs = [plotArgs, {imuTimeStamp, imuGyroX, ...
                                       imuTimeStamp, imuGyroY, ...
                                       imuTimeStamp, imuGyroZ}];
                plotColor = [plotColor; {obj.COLOR_IMU_GYRO_X; obj.COLOR_IMU_GYRO_Y; obj.COLOR_IMU_GYRO_Z}];
                plotLegend = [plotLegend; {'IMU GYRO X'; 'IMU GYRO Y'; 'IMU GYRO Z'}];
            end
            
            if any(strcmp(whatToPlot, 'ImuAccl'))
                assert(~isempty(obj.imuSampList), 'No imu data');
                plotArgs = [plotArgs, {imuTimeStamp, imuAcclX, ...
                                       imuTimeStamp, imuAcclY, ...
                                       imuTimeStamp, imuAcclZ}];
                plotColor = [plotColor; {obj.COLOR_IMU_ACCL_X; obj.COLOR_IMU_ACCL_Y; obj.COLOR_IMU_ACCL_Z}];
                plotLegend = [plotLegend; {'IMU ACCL X'; 'IMU ACCL Y'; 'IMU ACCL Z'}];
            end
            
            if ~isempty(plotArgs)
                h = plot(plotArgs{:});
                set(h, {'Color'}, plotColor);
                set(h, {'DisplayName'}, plotLegend);
                legend('-DynamicLegend');
            end
            
        end
        
        function PlotTimeStamp(obj, varargin)
            whatToPlot = unique(varargin);
            figure, hold on
            if any(strcmp(whatToPlot, 'ImuTimeStamp'))
                plot(obj.imuSampList(:,obj.IMUSAMP_ID), obj.imuSampList(:,obj.IMUSAMP_TIMESTAMP), 'g', obj.imuSampList(:,obj.IMUSAMP_ID), obj.imuSampList(:,obj.IMUSAMP_SYSTIMESTAMP), 'g--');
            end
            if any(strcmp(whatToPlot, 'CameraTimeStamp'))
                plot(obj.vidFrameInfoList(:,obj.CAMFRAMEINFO_ID), obj.vidFrameInfoList(:,obj.CAMFRAMEINFO_TIMESTAMP), 'r', obj.vidFrameInfoList(:,obj.CAMFRAMEINFO_ID), obj.vidFrameInfoList(:,obj.CAMFRAMEINFO_SYSTIMESTAMP), 'r--');
            end
        end
    end
    
    
end