classdef BodySensorInputInterface < handle
    
    methods (Abstract)
        Open(obj);
        Close(obj);
        
        GetNextDataFrame(obj);
        GetBodySensorDataDecoder(obj);
    end
    
end