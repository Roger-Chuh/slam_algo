
function main(folderName, keySeqNum, varargin)
% close all
% clc

if (nargin <= 2)
    baseDir = pwd;
elseif (nargin == 3)
    baseDir = varargin{1};
else
    error('Too many input arguments');
end



if ~isempty(folderName)
    MakeDirIfMissing(folderName);
end
% SetupRobotEnv('temp')
SetupRobotEnv('simulation_short_baseLine2')
% SetupRobotEnv('real1')
% SetupRobotEnv('reality3')
% TraceAndLocalize1();
% SLAM_Client(folderName);
ReplayTrace(folderName, keySeqNum, baseDir);
% ReplayTraceLite(folderName);