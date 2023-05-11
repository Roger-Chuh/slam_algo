function [cbcX, cbcY] = DetectCbCorner(cbImg)
% DetectCbCorner detect corner points of a chessboard image. It's based on
% "A Toolbox for Automatic Calibration of Range and Camera Sensors using a
% single Shot" by Andreas Geiger and Frank Moosmann and Omer Car and
% Bernhard Schuster
% cbImg is the chessboard image
% Returned cbcX and cbcY are all M by N matrix with each entry the x and y
% coordinates of the M by N corners of the chessboard
% By Ji Zhou

corners = findCorners(cbImg,0.01,1);
chessboards = chessboardsFromCorners(corners);

nCand = length(chessboards);
minDim = zeros(1, nCand);
for iCand = 1:nCand
    x = corners.p(chessboards{iCand}, 1);
    y = corners.p(chessboards{iCand}, 1);
    dx = max(x) - min(x);
    dy = max(y) - min(y);
    minDim(iCand) = min(dx,dy);
end
[~,ind] = max(minDim);
[rCorn, cCorn] = size(chessboards{ind});
cbcX = reshape(corners.p(chessboards{ind}, 1), rCorn, cCorn);
cbcY = reshape(corners.p(chessboards{ind}, 2), rCorn, cCorn);

end

