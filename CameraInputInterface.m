classdef CameraInputInterface < handle
    
    methods (Abstract)
        Open(obj);
        Close(obj);
        GetNextFrame(obj);
        HasFrame(obj);
    end
    
end