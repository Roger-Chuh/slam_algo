function read_board_depth(scrdirname,datdirname)
srcdirname = 'D:\Temp\20201112\record';
dstdirname = 'D:\Temp\20201112\record\1';

srcdirname = 'D:\Temp\20201114\raw\raw';
dstdirname = 'D:\Temp\20201114\raw\raw\1';

if ~exist(dstdirname, 'dir')
    mkdir(dstdirname);
end

subdirs = dir(fullfile(srcdirname,'depth_src*.raw'));
for i = 1 : length(subdirs)/2
    if strcmp(subdirs(i).name, '.') || strcmp(subdirs(i).name, '..')
        continue
    end   
    subnameL = fullfile(srcdirname, subdirs(i).name);
    numstrL = subdirs(i).name(end-7:end-4);
    depth_imgL = getImg(subnameL);
    subnameR = fullfile(srcdirname, subdirs(i + length(subdirs)/2).name);
    numstrR = subdirs(i + length(subdirs)/2).name(end-7:end-4);
    depth_imgR = getImg(subnameR);
    
%     filename= fullfile(dstdirname, ['depth_',numstr,'.png']);
    imwrite(uint8(depth_imgL), fullfile(dstdirname,sprintf('imgL_%05d.png', i)));
    imwrite(uint8(depth_imgR), fullfile(dstdirname,sprintf('imgR_%05d.png', i)));
end
end
function depth_img = getImg(subname)
disp(subname);
fp=fopen(subname,'rb');
%     depth= fread(fp, 'uint16=>uint16');
depth= fread(fp, 'uint8');
depth_img=reshape(depth(:),[480,640])';
end