function coordSysAlignParam = ReadCoordSysAlignDataDump

global DUMP_ROOT
dumpFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');

[dumpFid, errMsg] = fopen(dumpFilePath);
if (dumpFid < 0)
    error('Cannot open %s for read: %s', dumpFilePath, errMsg);
end

coordSysAlignParam(1) = ReadOneCamCoordSysAlignParam(dumpFid);

coordSysAlignParam(2) = ReadOneCamCoordSysAlignParam(dumpFid);


fclose(dumpFid);

end

function alignParam = ReadOneCamCoordSysAlignParam(dumpFid)

tempR = rodrigues(fscanf(dumpFid, '%f', 3));
tempt = fscanf(dumpFid, '%f', 3);
tempT = inv([tempR tempt;0 0 0 1]);
alignParam.rotMatB2C = tempT(1:3,1:3);
alignParam.transVecB2C = tempT(1:3,4);

extraDataFlag = fscanf(dumpFid, '%d', 1);
if (extraDataFlag == 1)
    alignParam.vanishPt = fscanf(dumpFid, '%f', 2);
    alignParam.vanishLn = fscanf(dumpFid, '%f', 3);
    
    alignParam.bodyRotPred.hCoef = fscanf(dumpFid, '%f', 3);
    alignParam.bodyRotPred.h0 = fscanf(dumpFid, '%f', 1);
    alignParam.bodyRotPred.hStep = fscanf(dumpFid, '%f', 3);
    alignParam.bodyRotPred.rMat = reshape(fscanf(dumpFid, '%f', 9), 3, 3)';
    alignParam.bodyRotPred.rOff = fscanf(dumpFid, '%f', 3);
    alignParam.bodyRotPred.r0 = fscanf(dumpFid, '%f', 1);
    alignParam.bodyRotPred.rStep = fscanf(dumpFid, '%f', 1);
    alignParam.bodyRotPred.numRows = fscanf(dumpFid, '%d', 1);
    alignParam.bodyRotPred.tableSize = fscanf(dumpFid, '%d', 1);
    alignParam.bodyRotPred.digits = fscanf(dumpFid, '%d', 3);
    alignParam.bodyRotPred.lut = fscanf(dumpFid, '%f', 3 * alignParam.bodyRotPred.tableSize);
    
    alignParam.camRotPred.rotAx = fscanf(dumpFid, '%f', 3);
    alignParam.camRotPred.cosTheta0 = fscanf(dumpFid, '%f', 1);
    alignParam.camRotPred.cosThetaStep = fscanf(dumpFid, '%f', 1);
    alignParam.camRotPred.numCoeffs = fscanf(dumpFid, '%d', 1);
    alignParam.camRotPred.digits = fscanf(dumpFid, '%d', 3);
    alignParam.camRotPred.lut = fscanf(dumpFid, '%f', 3 * alignParam.camRotPred.numCoeffs);
end

end