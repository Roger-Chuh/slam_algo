function [cbcX, cbcY] = CbCoordSys(cbcX, cbcY)

% Determine coordinate system attached to a checker board and rearrange the
% its corner point array.
% cbcX and cbcY are coordinates of checker board corner point array. They
% are matrices of same size.
% This function uses simple rule to determine origin point, x and y axes
% and their orientations: the direction with bigger column delta is X axis,
% and X axis points to column increasing direction. The another board of
% corner array is Y axis pointing to row increasing direction.
%
% By Ji Zhou

tp = diff(cbcX(1,:)); tpp = diff(cbcX(:,1));





% % % % % % tempX = cbcX; tempY = cbcY;
% % % % % % firstColY = tempY(:,1);
% % % % % % [firstColYRe, idY] = sort(firstColY,'ascend');
% % % % % % cbcX = cbcX(idY,:);
% % % % % % cbcY = cbcY(idY,:);
% % % % % % 
% % % % % % firstRowX = tempX(1,:);
% % % % % % [firstRowXRe, idx] = sort(firstRowX,'ascend');
% % % % % % cbcX = cbcX(:,idx);
% % % % % % cbcY = cbcY(:,idx);


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
avgDltCol1 = mean2(diff(cbcX, 1, 2));
avgDltCol2 = mean2(diff(cbcY, 1, 2));

if (abs(avgDltCol2) > abs(avgDltCol1))
    cbcX = cbcX';
    cbcY = cbcY';
end

% % % % % % % % % tempX = cbcX; tempY = cbcY;
% % % % % % % % % firstColY = tempY(:,1);
% % % % % % % % % [firstColYRe, idY] = sort(firstColY,'ascend');
% % % % % % % % % cbcX = cbcX(idY,:);
% % % % % % % % % cbcY = cbcY(idY,:);
% % % % % % % % % 
% % % % % % % % % firstRowX = tempX(1,:);
% % % % % % % % % [firstRowXRe, idx] = sort(firstRowX,'ascend');
% % % % % % % % % cbcX = cbcX(:,idx);
% % % % % % % % % cbcY = cbcY(:,idx);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
avgDltCol = mean2(diff(cbcX, 1, 2));
if (avgDltCol < 0)
    cbcX = fliplr(cbcX);
    cbcY = fliplr(cbcY);
end

avgDltRow = mean2(diff(cbcY, 1, 1));
if (avgDltRow < 0)
    cbcY = flipud(cbcY);
    cbcX = flipud(cbcX);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % % % % % % % tempX = cbcX; tempY = cbcY;
% % % % % % % % % % % % firstColY = tempY(:,1);
% % % % % % % % % % % % [firstColYRe, idY] = sort(firstColY,'ascend');
% % % % % % % % % % % % cbcX = cbcX(idY,:);
% % % % % % % % % % % % cbcY = cbcY(idY,:);
% % % % % % % % % % % % 
% % % % % % % % % % % % firstRowX = tempX(1,:);
% % % % % % % % % % % % [firstRowXRe, idx] = sort(firstRowX,'ascend');
% % % % % % % % % % % % cbcX = cbcX(:,idx);
% % % % % % % % % % % % cbcY = cbcY(:,idx);
end