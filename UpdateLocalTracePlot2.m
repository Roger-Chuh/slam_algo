function UpdateLocalTracePlot2(obj, img)
%             imshow(img);
            if IsLocalTraceEmpty(obj)
                return;
            end
            
            hold on;
            [~, ~, activeInd] = GetActiveFeatures(obj);
            localTraceX = obj.localTrace.xGT(activeInd, :);
            localTraceY = obj.localTrace.yGT(activeInd, :);
%             assert(all(all(localTraceX > 0 & localTraceY > 0)), 'The back trace of active features must be valid');
            if (LocalTraceLength(obj) >= 2)
                plot(localTraceX', localTraceY', 'k-');
                
%                 loseTrackingFlag = true(NumLocalTraceFeatures(obj), 1);
%                 loseTrackingFlag(activeInd) = false;
%                 ptIcsPrev = GetFeatures(obj, -1);
%                 loseTrackingFlag(ptIcsPrev(:,1) < 0) = false;
%                 if any(loseTrackingFlag)
%                     plot(ptIcsPrev(loseTrackingFlag, 1), ptIcsPrev(loseTrackingFlag, 2), 'kx');
%                 end
            end
            plot(localTraceX(:, end), localTraceY(:, end), 'bo');
            hold off;
        end