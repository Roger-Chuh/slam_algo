
aviobj=VideoWriter(fullfile(pwd,'demo - 1 -bigscene.avi')); 
aviobj.FrameRate = 2;
open(aviobj);
done = false;
figNummm = 98989;


for nji = 1 : length(inlierId)
%     featId2check = nji; FeatTrackingError;
    ParticularFeatDirectVSAccum(sc, obj, LocalTrace, [nji;nji], [obj.accumP2CRef(end-(keyFrameLen-1):end) - obj.accumP2CRef(end-(keyFrameLen-1))], angSampRng, curUpdateRatio, depthGTInd_update, [], 0,0)
     figure(figNummm);% subplot(1,3,1);hold on;title(num2str(nji));
     saveas(gcf,'temp.png');
     frameImg111 = imread('temp.png');
    
    writeVideo(aviobj,imresize(frameImg111, 0.5));
end
close(aviobj);