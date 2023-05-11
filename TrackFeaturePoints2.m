function [predPtList, inTrackFlag00] = TrackFeaturePoints2( prvImg, curImg, prvPtToTrack,inTrackFlag0)
obj.configParam.win_width = 9;
obj.configParam.win_height = 9;
obj.configParam.max_level = 3;
obj.configParam.criteria_count = 30;
obj.configParam.criteria_eps = 0.01;
obj.configParam.min_eigen_threshold = 0.0001;
predPtList = zeros(length(inTrackFlag0),2);
inTrackFlag00 = zeros(length(inTrackFlag0),1);
[predPtList_, trackStatus] = TrackingImplement2(obj, prvImg, curImg, prvPtToTrack(inTrackFlag0,:));

%             [topMargin, leftMargin, bottomMargin, rightMargin] = GetPredictMargin(obj);
%             inBndFlag = predPtList(:, 1) >= leftMargin + 1 & ...
%                         predPtList(:, 1) <= Cols(curImg) - rightMargin & ...
%                         predPtList(:, 2) >= topMargin + 1 & ...
%                         predPtList(:, 2) <= Rows(curImg) - bottomMargin;
%             inTrackFlag = trackStatus & inBndFlag;
%
vldId = find(inTrackFlag0); 
inTrackFlag = trackStatus>0;
predPtList__ = predPtList_(trackStatus>0,:);
vldId_ = vldId(inTrackFlag);
inTrackFlag00(vldId_) = 1;
inTrackFlag00 = logical(inTrackFlag00);
predPtList(inTrackFlag00,:) = predPtList__;
end
function [predictedPt2, status] = TrackingImplement2(obj, prvImg, curImg, prvPtToTrack )
% OpenCV implementation
[predictedPt2, status] = OcvTrackFeaturePointsPyrLK(prvImg, curImg, prvPtToTrack, ...
    [obj.configParam.win_width, obj.configParam.win_height], ...
    obj.configParam.max_level, obj.configParam.criteria_count, obj.configParam.criteria_eps, obj.configParam.min_eigen_threshold);

end