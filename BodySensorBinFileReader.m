classdef BodySensorBinFileReader < BodySensorInputInterface
    properties (Constant)
        BODYSENSOR_HDR_MAGIC = hex2dec('20170920');
        
        CMDTYPE_FORWARD = 0;
        CMDTYPE_STOP = 1;
        CMDTYPE_TURN_L = 2;
        CMDTYPE_TURN_R = 3;
        CMDTYPE_OVERFLOW = 4;
        CMDTYPE_INVALID = 5;
        
        CMDSTATUS_START = 0;
        CMDSTATUS_STOP = 1;
        CMDSTATUS_INVALID = 2;
        
        BODYSENSORSAMP_ID = 1;
        BODYSENSORSAMP_TIMESTAMP = 2;
        BODYSENSORSAMP_SYSTIMESTAMP = 3;
        BODYSENSORSAMP_V = 4;
        BODYSENSORSAMP_W = 5;
 %       BODYSENSORSAMP_FRAMECNT = 6;
        BODYSENSORSAMP_SONAR_DATA = 6;
        
        COLOR_BODYSENSOR_V = [0,0,1]; % b
        COLOR_BODYSENSOR_W = [0,1,0]; % g
        
    end
    
    properties
        bodySensorSampFields 
        bodySensorSampList
        bodySensorRespList
        
        lastFrameEndSampInd;
        lastSyncTimeStamp;
        sampleRate;
    end

    methods
        function obj = BodySensorBinFileReader()
            obj.bodySensorSampFields = 6;  % ID (uint32), timestamp (uint32), systimestamp(uint32), v (float), w (float)
            obj.sampleRate = 25; % samples/s
            
            obj.bodySensorSampList = [];
            obj.bodySensorRespList = [];
            
            obj.lastFrameEndSampInd = 0;
            obj.lastSyncTimeStamp = [];
        end
        
        function Open(obj, bodySensorBinFilePath, bodySensorRespBinFilePath)
            global IMU_BODYSNESOR_RAW_DATA_FIG
            
            ReadBodySensorBinFile(obj, bodySensorBinFilePath);
            fprintf('Total %d bodysensor samples are read from %s\n', Rows(obj.bodySensorSampList), bodySensorBinFilePath);
            
            ReadBodySensorRespBinFile(obj, bodySensorRespBinFilePath);
            fprintf('Total %d bodysensor response records are read from %s\n', Rows(obj.bodySensorRespList), bodySensorRespBinFilePath);
            
            if ~isempty(IMU_BODYSNESOR_RAW_DATA_FIG)
                figure(IMU_BODYSNESOR_RAW_DATA_FIG),subplot(2,1,1), PlotSamples(obj, 'BodySensorV'), hold on;
                figure(IMU_BODYSNESOR_RAW_DATA_FIG),subplot(2,1,2), PlotSamples(obj, 'BodySensorW'), hold on;
            end
        end
        
        function Close(obj)
            obj.bodySensorSampList = [];
            obj.bodySensorRespList = [];
            obj.lastFrameEndSampInd = 0;
            obj.lastSyncTimeStamp = [];
        end
        
        function ReadBodySensorBinFile(obj, bodySensorBinFilePath)
            assert(isempty(obj.bodySensorSampList) && isempty(obj.bodySensorRespList), 'Body sensors have already been opened. Close them first');
            [bodySensorBinFid, errMsg] = fopen(bodySensorBinFilePath);
            if (bodySensorBinFid <= 0)
                error('Cannot open %s for read: %s', bodySensorBinFilePath, errMsg);
            end
            
            packetCnt = 0;
            while (~feof(bodySensorBinFid))
                magicNum = fread(bodySensorBinFid, 1, 'uint32');
                if isempty(magicNum)
                    assert(feof(bodySensorBinFid) ~= 0, 'Incomplete bodySensorBinFile at packet %d', packetCnt);
                    break;
                end
                if magicNum ~= obj.BODYSENSOR_HDR_MAGIC
                    warning('Doesnot match magic number at packet %d', packetCnt)
                    break;
                end
%                 assert(magicNum == obj.BODYSENSOR_HDR_MAGIC, 'Doesnot match magic number at packet %d', packetCnt);
                

                numPktSamp = fread(bodySensorBinFid, 1, 'uint32');
                if isempty(numPktSamp)
                    warning('Incomplete bodySensorBinFile at packet %d', packetCnt)
                    break;
                end
%                 assert(~isempty(numPktSamp), 'Incomplete bodySensorBinFile at packet %d', packetCnt);
                
                [bodySensorPktData, rdCnt] = fread(bodySensorBinFid, numPktSamp * obj.bodySensorSampFields, 'single');
                if rdCnt ~= numPktSamp * obj.bodySensorSampFields
                    warning('Incomplete bodySensorBinFile at packet %d', packetCnt)
                    break;
                end
%                 assert(rdCnt == numPktSamp * obj.bodySensorSampFields, 'Incomplete bodySensorBinFile at packet %d', packetCnt);
                bodySensorPktData = reshape(bodySensorPktData, obj.bodySensorSampFields, [])';
                intFldIdx = [obj.BODYSENSORSAMP_ID, obj.BODYSENSORSAMP_TIMESTAMP, obj.BODYSENSORSAMP_SYSTIMESTAMP];
                bodySensorPktData(:,intFldIdx) = reshape(typecast(Vec(single(bodySensorPktData(:, intFldIdx))), 'uint32'), [], length(intFldIdx));
                
                obj.bodySensorSampList = [obj.bodySensorSampList; bodySensorPktData];
                
                packetCnt = packetCnt + 1;
            end
            
            fclose(bodySensorBinFid);
        end
        
        function ReadBodySensorRespBinFile(obj, bodySensorRespBinFilePath)
            assert(isempty(obj.bodySensorRespList), 'Body sensors have already been opened. Close them first');
            [bodySensorRespBinFid, errMsg] = fopen(bodySensorRespBinFilePath);
            if (bodySensorRespBinFid <= 0)
                error('Cannot open %s for read: %s', bodySensorRespBinFilePath, errMsg);
            end
            
            % [cmdstatus, cmdtype, timestamp, ackdata]
            obj.bodySensorRespList = reshape(fread(bodySensorRespBinFid, inf, 'uint32'), 4, [])';
            
            fclose(bodySensorRespBinFid);
        end
        
        function [dataFrame, numLostSamp] = GetNextDataFrame(obj, vidFrameTimeStamp, timeOffset)
            currFrameEndSampInd = find(obj.bodySensorSampList(:, obj.BODYSENSORSAMP_SYSTIMESTAMP) + timeOffset >= vidFrameTimeStamp, 1, 'first');
            if isempty(currFrameEndSampInd)
                dataFrame = [];
                numLostSamp = [];
                return;
            end
            assert(sum(obj.bodySensorSampList(:, obj.BODYSENSORSAMP_TIMESTAMP) == obj.bodySensorSampList(currFrameEndSampInd, obj.BODYSENSORSAMP_TIMESTAMP))==1, ...
                   'Duplicated body sensor timestamp detected.');
            dataFrame = obj.bodySensorSampList(obj.lastFrameEndSampInd + 1:currFrameEndSampInd, :);
            
            if (obj.lastFrameEndSampInd == 0)
                numLostSamp = obj.bodySensorSampList(currFrameEndSampInd, obj.BODYSENSORSAMP_ID) - obj.bodySensorSampList(1, obj.BODYSENSORSAMP_ID) + 1 - Rows(dataFrame);
            else
                numLostSamp = obj.bodySensorSampList(currFrameEndSampInd, obj.BODYSENSORSAMP_ID) - obj.bodySensorSampList(obj.lastFrameEndSampInd, obj.BODYSENSORSAMP_ID) - Rows(dataFrame);
            end
            
            obj.lastFrameEndSampInd = currFrameEndSampInd;
            obj.lastSyncTimeStamp = vidFrameTimeStamp;
        end
        
        function PlotSamples(obj, varargin)
            whatToPlot = unique(varargin);
            
            if ~isempty(obj.bodySensorSampList)
                bodySensorTimeStamp = obj.bodySensorSampList(:, obj.BODYSENSORSAMP_SYSTIMESTAMP);
                bodySensorV = obj.bodySensorSampList(:,obj.BODYSENSORSAMP_V);
                bodySensorW = obj.bodySensorSampList(:,obj.BODYSENSORSAMP_W) * 180 / pi;
            end
            
            plotColor = [];
            plotLegend = [];
            plotArgs = [];
            if any(strcmp(whatToPlot, 'BodySensorV'))
                assert(~isempty(obj.bodySensorSampList), 'No body sensor data');
                plotArgs = [plotArgs, {bodySensorTimeStamp, bodySensorV}];
                plotColor = [plotColor; {obj.COLOR_BODYSENSOR_V}];
                plotLegend = [plotLegend; {'BodySensor V'}];
            end
            if any(strcmp(whatToPlot, 'BodySensorW'))
                assert(~isempty(obj.bodySensorSampList), 'No body sensor data');
                plotArgs = [plotArgs, {bodySensorTimeStamp, bodySensorW}];
                plotColor = [plotColor; {obj.COLOR_BODYSENSOR_W}];
                plotLegend = [plotLegend; {'BodySensor W'}];
            end

            if ~isempty(plotArgs)
                h = plot(plotArgs{:});
                set(h, {'Color'}, plotColor);
                set(h, {'DisplayName'}, plotLegend);
                legend('-DynamicLegend');
            end
        end
        
        function dec = GetBodySensorDataDecoder(obj)
            dec = struct('ID', obj.BODYSENSORSAMP_ID, ...
                         'TIMESTAMP', obj.BODYSENSORSAMP_TIMESTAMP, ...
                         'SYSTEMTIMESTAMP', obj.BODYSENSORSAMP_SYSTIMESTAMP, ...
                         'V', obj.BODYSENSORSAMP_V, ...
                         'W', obj.BODYSENSORSAMP_W);
        end
    end
end