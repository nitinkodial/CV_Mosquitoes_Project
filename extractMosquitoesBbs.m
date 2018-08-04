%extract mosquitoes bbs from lab videos and corresponding tracks and save it to file and also add new shifted bbs to allSavedTracks
%inputs-original video,mask video, track.mat
function extractMosquitoesBbs()


root_folder = '/home/nitin/gatech_courses';


videoName = 'MVI_7500'
videoPath1 = strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/mov/',videoName);%videoName,'.MOV');
videoPath2 = strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/masks/',videoName,'_output_multi_BG.avi');
trackPath = strcat(root_folder,'/special_problem/code_nitk/tracks/working_tracks/',videoName,'_output_multi.mat');
trackPath2 = strcat(root_folder,'/special_problem/code_nitk/tracks/working_tracks/',videoName,'_output_multi_withShifts.mat');

frameNum=0;
numOfFramesInVideo = 400;%number of video frames in folder
runNFrames = numOfFramesInVideo;
runTillEndFlag = true;
%skipping does not work, due to frameNums mismatch
skipNFrame = 0;
frameRate = 30;%fps
skipNSeconds = skipNFrame/frameRate;

% Create a video file reader1.
%  reader1 = Videoreader1(strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/mp4/',char(input_video_files(varI)),'.mp4'),'CurrentTime',skipNSeconds);

reader1 = VideoReader(videoPath1,'CurrentTime',skipNSeconds);

% Create a video file reader2.
%  reader2 = Videoreader1(strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/mp4/',char(input_video_files(varI)),'.mp4'),'CurrentTime',skipNSeconds);
reader2 = VideoReader(videoPath2,'CurrentTime',skipNSeconds);


%  writer = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/PRG1_A06I8031_withBbs_v1'));%appends .avi automa%tically

%  open(writer);

videoPlayer = vision.VideoPlayer('Position', [0,0,1920,1080]);

allSavedTracks = [];

%load allSavedTracksOutputResized
load(trackPath);

allSavedTracksOutput = allSavedTracks;

%create new field called all_bboxes_shifted to hold shifted bbs:
[allSavedTracksOutput(:).all_bboxes_shifted]=deal(zeros(1,4));

numOfTracks = size(allSavedTracksOutput,2);

trackStartIndices = [];
for varI = 1:numOfTracks
    %startFrameNum,endFrameNum,trackId,trackIndex
    trackStartIndices = cat(1, trackStartIndices, [allSavedTracksOutput(varI).frameNums(1), allSavedTracksOutput(varI).frameNums(end), allSavedTracksOutput(varI).id, varI] );
    allSavedTracksOutput(varI).all_bboxes_shifted = zeros(allSavedTracksOutput(varI).totalVisibleCount,4);
    for varJ = 1:(allSavedTracksOutput(varI).totalVisibleCount)
        if(varJ)==1
            allSavedTracksOutput(varI).all_bboxes_shifted(varJ,1:2) = 1;
            allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4) = allSavedTracksOutput(varI).all_bboxes(varJ,3:4);
        else
            allSavedTracksOutput(varI).all_bboxes_shifted(varJ,1) = allSavedTracksOutput(varI).all_bboxes(varJ,1)-allSavedTracksOutput(varI).all_bboxes(varJ-1,1)+allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,1);
            allSavedTracksOutput(varI).all_bboxes_shifted(varJ,2) = allSavedTracksOutput(varI).all_bboxes(varJ,2)-allSavedTracksOutput(varI).all_bboxes(varJ-1,2)+allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,2);
            allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4) = allSavedTracksOutput(varI).all_bboxes(varJ,3:4);
        end
        
    end
end

allSavedTracks = allSavedTracksOutput;

%save tracksMat to file
save(trackPath2,'allSavedTracks');
%  return


trackStartIndices = sortrows(trackStartIndices);
trackStartIndices;
nextId = 1; % ID of the next track



activeTrackIdNFrameNum = [];

startTrackCounter = 1;
endTrackCounter = 1;

while hasFrame(reader1) & hasFrame(reader2)
    
    %rgb frame
    frame1 = readFrame(reader1);
    
    %BG frame
    frame2 = readFrame(reader2);

    frameNum = frameNum + 1
    
    %close processing early
    if(frameNum==runNFrames&&runTillEndFlag==false)
        break
    end

    %open processing after skipping n frames
    if(frameNum<=skipNFrame)
        continue
    end
    
    if(startTrackCounter<=size(trackStartIndices,1))
        while(frameNum == trackStartIndices(startTrackCounter,1))
            %store active track id,currFrameNum,startFrameNum for Track, endFrameNum for Track, trackIndex, frameCounter
            activeTrackIdNFrameNum = cat(1,activeTrackIdNFrameNum,[trackStartIndices(startTrackCounter,3),frameNum, trackStartIndices(startTrackCounter,1), trackStartIndices(startTrackCounter,2), trackStartIndices(startTrackCounter,4), 1]);
            startTrackCounter = startTrackCounter + 1;
            trackStartIndices;
            if(startTrackCounter>size(trackStartIndices,1))
                break
            end
        end
    end
    
    displaySavedTrackingResults()
    
    idsToBeRemoved = [];
    for varI = 1:size(activeTrackIdNFrameNum,1)
        activeTrackIdNFrameNum(varI,2) = activeTrackIdNFrameNum(varI,2) + 1;
        if(activeTrackIdNFrameNum(varI,2)>activeTrackIdNFrameNum(varI,4))
            idsToBeRemoved = [idsToBeRemoved,varI];
        end
    end
    activeTrackIdNFrameNum(idsToBeRemoved,:)=[];
    activeTrackIdNFrameNum;
    
end

%  close(writer)

    function displaySavedTrackingResults()
        
        bboxes = [];
        labels = [];
        labels_char = [];

        widthOfFrame = size(frame1,2);
        heightOfFrame = size(frame1,1);
        
        for varI = 1:size(activeTrackIdNFrameNum,1)
            
            currTrackIndex = activeTrackIdNFrameNum(varI,5);
%              currBbsId = activeTrackIdNFrameNum(varI,2) - activeTrackIdNFrameNum(varI,3) + 1;
            if(frameNum==allSavedTracksOutput(currTrackIndex).frameNums(activeTrackIdNFrameNum(varI,6)))  
                currBbs = allSavedTracksOutput(currTrackIndex).all_bboxes((activeTrackIdNFrameNum(varI,6)),:);
            
                if(size(currBbs,1)>0)
                    expand_bbs_size = 0;%18;
                    
                    currBbs(:,1) =  currBbs(:,1) - expand_bbs_size/2;
                    if(currBbs(:,1)<1)
                        currBbs(:,1) = 1
                    end
                    
                    currBbs(:,2) =  currBbs(:,2) - expand_bbs_size/2;
                    if(currBbs(:,2)<1)
                        currBbs(:,2) = 1
                    end

                    currBbs(:,3) =  currBbs(:,3) + expand_bbs_size;
                    if((currBbs(:,1)+currBbs(:,3)-1)>widthOfFrame)
                        currBbs(:,3) = widthOfFrame + 1 - currBbs(:,1);
                    end
                    
                    currBbs(:,4) =  currBbs(:,4) + expand_bbs_size;
                    if((currBbs(:,2)+currBbs(:,4)-1)>heightOfFrame)
                        currBbs(:,4) = heightOfFrame + 1 - currBbs(:,2);
                    end
                    
                    bboxes = cat(1,bboxes,currBbs);
                    id = int32(activeTrackIdNFrameNum(varI,1));
%                      bboxes
%                      id
%                      disp('d0')
                    labels = cat(1,labels,cellstr(int2str(id)));
                    labels_char = cat(1,labels_char,(int2str(id)));
%                      labels
                end
                
                activeTrackIdNFrameNum(varI,6) = activeTrackIdNFrameNum(varI,6) + 1;

            end
            
        end
        
        if ~isempty(bboxes)
                %crop and save bbs to file:
                for varI = 1:size(bboxes)
                    bbs_rgb_img = imcrop(frame1,bboxes(varI,:));
                    bbs_bg_img = imcrop(frame2,bboxes(varI,:));
%                      bboxes
%                      id
%                      size(bboxes)
%                      size(id)
%                      id(varI)
%                      labels_char
                    strcat(root_folder,'/special_problem/code_nitk/mosquito_bbs/',videoName,'/rgb/',videoName,'_',int2str(frameNum),'_',labels_char(varI,:))
                    imwrite(bbs_rgb_img,strcat(root_folder,'/special_problem/code_nitk/mosquito_bbs/',videoName,'/rgb/',videoName,'_',int2str(frameNum),'_',labels_char(varI,:)),'PNG');
                    imwrite(bbs_bg_img,strcat(root_folder,'/special_problem/code_nitk/mosquito_bbs/',videoName,'/masks/',videoName,'_',int2str(frameNum),'_',labels_char(varI,:)),'PNG');
                end
            
                % Draw the objects on the frame.%Uncomment when you want to display frames
%                  frame1 = insertObjectAnnotation(frame1, 'rectangle', ...
%                              bboxes, labels);
                                
        end
        
%Uncomment when you want to display frames       
%          videoPlayer.step(frame1);
        %30 ms pause
%          pause(0.03);

%          k = waitforbuttonpress;
%          pause;
    
%          writeVideo(writer,frame1);
        
    end
end