function RefineZZ(obj,intrMat,featPtManager,coordSysAligner,k2cRef,keyFrameFlagList,frmStampList,goldenPose)

[keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
            maxk2cRotAng = [k2cRef k2cRef] + [-obj.configParam.pnp_ang_est_max_margin obj.configParam.pnp_ang_est_max_margin];
            minAngGrid = obj.configParam.pnp_ang_est_min_angle_grid;
            k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
            numRotAng = length(k2cBodyRotAngCand);
            reProjErr = zeros(numRotAng,1);



end