function readYUVSingle( inputDir ,width, height,scale, name)



global cfg
if length(scale) == 1
    scale = [scale,scale];
end
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

try
    cfg.check_depth;
catch
    cfg.check_depth = 0;
end

% close all
if ~cfg.check_depth
    dirInfo = dir(fullfile(inputDir,'*.yuv'));
    if length(dirInfo) == 0
        dirInfo = dir(fullfile(inputDir,'*.bin'));
    end
else
    read_board_depth(inputDir, width, height);
    return;
end


if 0
    for i = 1 : length(dirInfo)/2
        id_ = find(dirInfo(i).name == '_');
        timeNum1(i,1) = str2double(dirInfo(i).name(id_(1)+1:id_(2)-1));
        
    end
    
    for i = length(dirInfo)/2 + 1 : length(dirInfo)
        id_ = find(dirInfo(i).name == '_');
        timeNum2(i - length(dirInfo)/2,1) = str2double(dirInfo(i).name(id_(1)+1:id_(2)-1));
        
    end
end


dircell=struct2cell(dirInfo);
aaAa = dircell(2,:)';
%     sss = datetime(aaAa,'Format','HH:mm:ss','Locale','zh_CN');

if 1
    if 1
        if 1 
            %% 20201112 big bug,(change ## if 0 -> if 1 ##) why wait till just now!!!
            [~, ind1] = sort([dirInfo(1:length(dirInfo)/1).datenum], 'ascend');
%             [~, ind2] = sort([dirInfo(length(dirInfo)/2+1:end).datenum], 'ascend');
%             ind2 = ind2 + length(dirInfo)/2;
%             I = [ind1 ind2];
            I = [ind1];
        else
            [~, ind1] = sort([timeNum1'], 'ascend');
            [~, ind2] = sort([timeNum2'], 'ascend');
            ind2 = ind2 + length(dirInfo)/2;
            I = [ind1 ind2];
            
        end
    else
        sss = datetime(aaAa,'Format','dd-MMM-y HH:mm:ss');
        [B,I] = sort(sss);
        
    end
    
    tempInfo = cell(length(I),1);
    for fg = 1:length(I)
        tempInfo{fg,1} = dirInfo(I(fg)).name;
    end
    for fgg = 1:length(I)
        dirInfo(fgg).name = tempInfo{fgg};
    end
end

outFlag = [];
% % outFlag = [1 2 17 19 20 21 22 37 39 40 41 42  43 60];
% % outFlag = [3 4 6 7 8 11 13 14 15 17 19 20 23 24 27 28 35 36 37 38 40 41 45 48 49 52 53 56 57 58 61 67 68 70 71 73 75 76];
outFlag = []; %setdiff([1:40]',[1;2;3;5;6;8;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40]);
frameNum = length(dirInfo)/1;

if 0
    dirInfo = [dirInfo(1:2:end);dirInfo(2:2:end)];
end

thr = 130;
H = fspecial('gaussian',21);

for i =    1 : frameNum  %[1 2 4 6 8 13 14 16 18 20 25 26 28 30 35 36 37 39 41 47 48 49 51 53 55 58 60 61 63 65 67]
    %     if 0
    if 0
        fid1=fopen(fullfile(inputDir,dirInfo(i).name),'rb');
        % % % % %     nameList(i,:) = dirInfo(i).name([14 15 16]);
        % % % % %     if strcmp(nameList(end),'_');
        % % % % %         numList(i,:) = str2double(strsplit(strcat(dirInfo(i).name([14 15]),',',dirInfo(i).name([17])),','));
        % % % % %     else
        % % % % %         numList(i,:) = str2double(strsplit(strcat(dirInfo(i).name([14]),',',dirInfo(i).name([16])),','));
        % % % % %     end
        
        A = fread(fid1,'uint8');
        AA = reshape(A(1:width*height),width,height)';
        % %     figure,imshow(AA,[])
        fclose(fid1);
        
        
        
        fid2=fopen(fullfile(inputDir,dirInfo(i + frameNum).name),'rb');
        B = fread(fid2,'uint8');
        BB = reshape(B(1:width*height),width,height)';
        
        
        % %     figure,imshow(AA,[])
        fclose(fid2);
    end
    % %         imwrite(imresize(uint8(AA),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgL_%05d.png',i)));
    % %         imwrite(imresize(uint8(BB),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgR_%05d.png',i)));
    %     else
    CC = dispI420(fullfile(inputDir,dirInfo(i).name),width,height);
    if 0
        CC(CC > 100) = 0;
        CC(:,:,1) = 255.*CC(:,:,1)./max(max((CC(:,:,1))));
        CC(:,:,2) = 255.*CC(:,:,2)./max(max((CC(:,:,2))));
        CC(:,:,3) = 255.*CC(:,:,3)./max(max((CC(:,:,3))));
    end
    if 0
        DD = dispI420(fullfile(inputDir,dirInfo(i + frameNum).name),width,height);
    end
    if 0
        DD = 255.*DD./max(DD(:));
    end
    % %         imwrite(imresize(uint8(AA),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgL_%05d.png',i)));
    % %         imwrite(imresize(uint8(BB),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgR_%05d.png',i)));
    
    
    if 0
        CC = imfilter(uint8(CC), H);
        DD = imfilter(uint8(DD), H);
    elseif 0
        CC = imgaussfilt(uint8(CC), 1.5);
%         DD = imgaussfilt(uint8(DD), 1.4);
    else
        sadghk = 1;
    end
    if 0
        CC = Precess(uint8(CC), thr);
        DD = Precess(uint8(DD), thr);
        
%         figure(1),imshow([CC DD]);
%         drawnow;
    end
    asrgkj = 1;
    if  1 % mod(i,2) == 0
        imwrite(imresize(uint8(CC),[height/scale(1),width/scale(2)]), fullfile(inputDir,sprintf(strcat('img',name,'_%04d.png'),i)));
%         imwrite(imresize(uint8(DD),[height/scale(1),width/scale(2)]), fullfile(inputDir,sprintf('imgR_%05d.png',i)));
    end
    
    %     end
    % %     imwrite(imresize((AA),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgL_%05d.png',i)));
    % %     imwrite(imresize((BB),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgR_%05d.png',i)));
    
    
    aadsf = 1;
    
    % %     if i <= length(dirInfo)/2
    % %         if ~ismember(i,outFlag)
    % %             imwrite(imresize(uint8(AA),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgL_%05d.png',i)));
    % %         end
    % %     else
    % %         if ~ismember(i-length(dirInfo)/2,outFlag)
    % %             imwrite(imresize(uint8(AA),[n/scale,m/scale]), fullfile(inputDir,sprintf('imgR_%05d.png',i-length(dirInfo)/2)));
    % %         end
    % %     end
    
end



% % for i = 1 : length(dirInfo)/2
% %     id(i,1) = str2double(dirInfo(i).name(6:end-4));
% % end
end
function rgbImg = dispI420(inputfile,width,height)
fd = fopen(inputfile, 'rb');
if fd == -1
    error('open file error');
end
% width = 1920;
% height = 1080;
y = fread(fd, width*height,'uint8');
u = fread(fd, width*height/4,'uint8');
v = fread(fd, width*height/4,'uint8');
fclose(fd);
yimg = reshape(y, [width,height]);
uimg = reshape(u, [width/2,height/2]);
vimg = reshape(v, [width/2,height/2]);
if 0
    rgbImg = Yuv420P2Rgb(yimg', uimg', vimg');
else
    rgbImg = cat(3, yimg', yimg', yimg');
end
% rgbImg = im2frame(uint8(rgbImg));
% rgbImg = (uint8(rgbImg));
% % figure(1),movie(fg);
end
function rgbImg = Yuv420P2Rgb(y, u, v)
% R = Y + 1.402 * (V - 128);
% G = Y - 0.34414 * (U - 128) - 0.71414 * (V - 128);
% B = Y + 1.772 * (U - 128);
% I420: YYYYYYYY UU VV =>YUV420P
% YV12: YYYYYYYY VV UU =>YUV420P
% 4:1:1

rImg = zeros(size(y));
gImg = zeros(size(y));
bImg = zeros(size(y));
rImg(1:2:end,1:2:end) = y(1:2:end,1:2:end) + 1.0402 * (v - 128);
rImg(2:2:end,1:2:end) = y(2:2:end,1:2:end) + 1.0402 * (v - 128);
rImg(2:2:end,2:2:end) = y(2:2:end,2:2:end) + 1.0402 * (v - 128);
rImg(1:2:end,2:2:end) = y(1:2:end,2:2:end) + 1.0402 * (v - 128);

gImg(1:2:end,1:2:end) = y(1:2:end,1:2:end) + 0.34414 * (u - 128) - 0.71414 * (v - 128);
gImg(2:2:end,1:2:end) = y(2:2:end,1:2:end) + 0.34414 * (u - 128) - 0.71414 * (v - 128);
gImg(2:2:end,2:2:end) = y(2:2:end,2:2:end) + 0.34414 * (u - 128) - 0.71414 * (v - 128);
gImg(1:2:end,2:2:end) = y(1:2:end,2:2:end) + 0.34414 * (u - 128) - 0.71414 * (v - 128);

bImg(1:2:end,1:2:end) = y(1:2:end,1:2:end) + 1.772 * (u - 128);
bImg(2:2:end,1:2:end) = y(2:2:end,1:2:end) + 1.772 * (u - 128);
bImg(2:2:end,2:2:end) = y(2:2:end,2:2:end) + 1.772 * (u - 128);
bImg(1:2:end,2:2:end) = y(1:2:end,2:2:end) + 1.772 * (u - 128);

rgbImg = zeros(size(y,1), size(y,2), 3);
rgbImg(:,:,1) = rImg;
rgbImg(:,:,2) = gImg;
rgbImg(:,:,3) = bImg;
end
function CC = Precess(CC, thr)

 CC = rgb2gray(CC);
 CC(CC > thr) = 0;
    CC = im2double(CC);
    CC = CC./max(CC(:));
    CC= uint8(255.*(CC));
    CC = cat(3,CC,CC,CC);

end