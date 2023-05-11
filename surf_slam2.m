function surf_slam2(frame_curr,k)

global Map
global State
global Params
global Debug

persistent frame_prev
% if ~isempty(frame_prev)
if  k > 1 

    structure_from_motion(frame_prev, frame_curr,k);

    
    if 0 % mod(k, Params.cullingSkip) == 0
        disp('Before keyframe culling')
        disp(Map.covisibilityGraph.NumViews);
        
        local_mapping();
        
        disp('After keyframe culling')
        disp(Map.covisibilityGraph.NumViews);
    end
    
	loop_closing();
else
	% Initialize
	[descriptors, points] = extract_features(frame_curr);

    bow = calc_bow_repr(descriptors, Params.kdtree, Params.numCodewords);

	Map.covisibilityGraph = addView(Map.covisibilityGraph, 1,...
        descriptors, points, bow, 'Points', points, ...
		'Orientation', eye(3), 'Location', zeros(1, 3));

end

	frame_prev = frame_curr;
end
