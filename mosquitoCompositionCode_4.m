%function to display downsized mosquitoe tracks on videos frames of a given dataset. This code uses joining of multiple tracks, to increase the area of tracks
%This composites 'n' mosquitoes onto the video per frame.
function MotionBasedMultiObjectTrackingExample()


enlargeBbsBy2x = true;
showMergedTrackNums = false;

input1_video_files = [string('PRG1')];%,string('PRG6'),string('PRG7'),string('PRG14'),string('PRG22'),string('PRG23'),string('PRG28'),string('PRG29')]; 
input2_video_files = [string('A06I8030')];%,string('A06I8031'),string('A06I8032'),string('A06I9821'),string('A06I9822'),string('A06I9823'),string('A06I9824')];


for varK1 = 1:size(input1_video_files,2) 
    for varK2 = 1:size(input2_video_files,2) 
        
        %to create reproducible results
        rng(varK1+varK2);
        
        % Create System objects used for reading video, detecting moving objects,
        % and displaying the results.
        obj = setupSystemObjects();

        %  tracks = initializeTracks(); % Create an empty array of tracks.
        allSavedTracks = [];
        load(obj.inputTracksFileName); %load allSavedTracks captured during entire video
        numOfTracks = size(allSavedTracks,2);
        %the tracks which will have bbs values updated to match the downscaled mosquitoes moving in the output video with randomness added to bbs.
        allSavedTracksOutput = allSavedTracks;

%          trackStartIndices = [];
%          for varI = 1:numOfTracks
%              %startFrameNum,endFrameNum,trackId,trackIndex
%              trackStartIndices = cat(1, trackStartIndices,[1,1,1,]);% [allSavedTracks(varI).frameNums(1), allSavedTracks(varI).frameNums(end), allSavedTracks(varI).id, varI] );
%          end
%          trackStartIndices = sortrows(trackStartIndices);
%          trackStartIndices;
%          nextId = 1; % ID of the next track
%          
%          %after combining the tracks,give each continuous track a number or id, and when the track breaks off and when a new track is started at a random edge, give that the next number as id.
%          [allSavedTracksOutput(:).trackId] = deal(1);
%          [allSavedTracksOutput(:).downScaleFactor] = deal(1);
%          [allSavedTracksOutput(:).all_bboxes_shifted_output] = deal(zeros(1,4));
%          [allSavedTracksOutput(:).startrow] = deal(1);
%          [allSavedTracksOutput(:).startcol] = deal(1);
%          
%          %Positions of where mosquito frames need to be placed
%          startrows = [200]; %[700,700,100,100,300];
%          startcols = [200]; %[1300,420,1300,1100,400];
%  
%          %reduce mosquitoes size by this factor:
%          downScaleFactors = [7.0]; %[5.0,10.0,20.0,22.0,15.0];
%          downScaleFactorsOriginal = downScaleFactors;     
%          mean_track_distance_x = 0;
%          mean_track_distance_y = 0;
%          
%          curr_counter = 1;
%          for varI = 1:size(allSavedTracksOutput,2)
%              allSavedTracksOutput(varI).all_bboxes_shifted_output = zeros(allSavedTracksOutput(varI).totalVisibleCount,4);
%              allSavedTracksOutput(varI).trackId = zeros(allSavedTracksOutput(varI).totalVisibleCount,1);
%              if(varI>1)
%                 
%              end
%              for varJ = 1:(allSavedTracksOutput(varI).totalVisibleCount)
%                  
%                  allSavedTracksOutput(varI).trackId(varJ) = curr_counter;
%                  %if first position of first track only,set to user defined startrow and
%                  %start col
%                  if(varJ==1&&varI==1)
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1:2) = [startrows(1),startcols(1)];
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactors(1));
%                  elseif(varJ==1&&varI>=1)
%                  %if first position of tracks after the first track,set to
%                  %previous track's last position + or - the mean track distance of
%                  %previous track (both in x direction and y direction), to enable smooth continuous tracks
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_distance_x*randi([-1,1]));
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_distance_y*randi([-1,1]));
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(1,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(1,3:4)/downScaleFactors(1));
%                  else
%  %                      varJ
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,1)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,1))/downScaleFactors(1)+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,1));
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,2)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,2))/downScaleFactors(1)+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,2));
%                      allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactors(1));
%                  end
%                  
%                  imgWidth = 704;
%                  imgHeight = 480;
%                  %if the current position of mosquito is outside of the
%                  %video region, randomly continue it at another edge, as
%                  %though a new mosquito entered into the video
%  %                 disp('d0');
%                  curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:);
%  %                 checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)
%                  if(checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)==true)
%                      curr_counter = curr_counter + 1;
%                      allSavedTracksOutput(varI).trackId(varJ) = curr_counter;
%                      %these distances are the mean distance the mosquito moved in x
%                      %and y direction at one instant. They will be used to jump from
%                      %one track to another track smoothly.
%                      [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance(allSavedTracksOutput(varI),downScaleFactors(1));
%                      %decide which side the mosquito enters from,
%                      %1-left,2-right,3-top or 4-bottom
%  %                     mean_track_distance_x
%  %                     mean_track_distance_y
%  %                     pause
%                      rand_edge = randi(4);
%                      %add this value to x or y position of reentry to
%                      %restart the track a bit further away from edges to prevent
%                      %frequent restarting of tracks, via frequenting
%                      %exiting.
%                      shift_random_entry = 20;
%                      if(rand_edge==1)
%                          rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = 1 + mean_track_distance_x + shift_random_entry;
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
%                      elseif(rand_edge==2)
%                          rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = imgWidth - mean_track_distance_x - curr_bbs(3) - shift_random_entry ;
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
%                      elseif(rand_edge==3)
%                          rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = 1 + mean_track_distance_y + shift_random_entry;   
%                      else
%                          rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
%                          allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = imgHeight - mean_track_distance_y - curr_bbs(4) - shift_random_entry;
%                      end
%                  end
%  %                 curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:)
%  %                 disp('d1');
%                  
%                  
%              end
%              %these distances are the mean distance the mosquito moved in x
%              %and y direction at one instant. They will be used to jump from
%              %one track to another track smoothly.
%              [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance(allSavedTracksOutput(varI),downScaleFactors(1));
%              
%  %              allSavedTracksOutput(varI).all_bboxes_shifted_output
%          end
%  %          allSavedTracksOutput(1).all_bboxes_shifted_output
%  %          allSavedTracksOutput(1).all_bboxes_shifted
%  
%          save('allSavedTracksOutput.mat','allSavedTracksOutput');


        numOfSplits = 5;%number of mosquitoes
        inputTrack = allSavedTracks;
        videoImgWidth = 704;
        videoImgHeight = 480;
        inputTrackName = 'A06I8030_';
        
        %reduce mosquitoes size by this factor:
        downScaleFactor = 7.0; 
        
        allSavedTracksSplitOutput = split_mosquito_tracks(inputTrack,inputTrackName,numOfSplits,downScaleFactor,videoImgWidth,videoImgHeight);
        
        
%  %          disp('return from here');
%  %          return 

        frameNum=0;
        numOfFramesInVideo = 3000;%number of video frames in folder
        runNFrames = numOfFramesInVideo;
        runTillEndFlag = false;
        skipNFrame = 0;



       

%          %Random shift value for ROI box in pixels
%          rand_shift_val = 5;

        %write video to file
        open(obj.writer1)
        open(obj.writer2)

        activeTrackIdNFrameNum = [];

        startTrackCounter = 1;
        endTrackCounter = 1;
%          trackCounter = [];

        numOfTrackSets = [];
        for varI =  1:numOfSplits
            numOfTrackSets = [numOfTrackSets;varI];
        end
        trackCounter = ones(numOfSplits);
        trackFrameCounter = ones(numOfSplits);
        allSavedTracksSplitOutput;
        
        % Detect moving objects, and track them across video frames.
        while ~isDone(obj.reader3)

            %tic
            %read video frames onto which mosquitoes have to be composited
            frame = readFrame2();
            %toc
       
            frameNum = frameNum + 1;
            if(frameNum==runNFrames&&runTillEndFlag==false)
                break
            end

            if(frameNum<=skipNFrame)
                continue
            end
            frameNum
            
            %tic
            displaySavedTrackingResults()
            %toc
            
            if(size(numOfTrackSets,2)==0)
                break;
            end
            
        end

        %Save any remaining tracks still active at end of video
        %  saveValidActiveTracksBeforeClose()

        %save resized tracksMat to file
%          save(obj.outputTracksFileName,'allSavedTracksOutput');


        %write video to file
        close(obj.writer1)
        close(obj.writer2)
    end
end

    function obj = setupSystemObjects()
        % Initialize Video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        root_folder = '/home/nitin/gatech_courses';
        % Create a video file reader.
        obj.reader1 = vision.VideoFileReader(strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/mp4/',char(input2_video_files(varK2)),'.mp4'));
        obj.reader2 = vision.VideoFileReader(strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/masks/',char(input2_video_files(varK2)),'_output_multi_BG.avi'));
        obj.reader3 = vision.VideoFileReader(strcat(root_folder,'/special_problem/code_nitk/videos/input/camnet_dataset/Videos/',char(input1_video_files(varK1)),'.avi'));
        
        %image reader %not used currently
        obj.image_reader = (strcat(root_folder,'/special_problem/lasiesta_database/I_BS_01/I_BS_01/I_BS_01-%d.bmp'));
        obj.input_image_number = 1;
        obj.final_image_number = 275;

        % Create 2 video file writers, one for color frame, another for bg frame.
        obj.writer1 = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input1_video_files(varK1)),'_',char(input2_video_files(varK2)),'_noBbs'));%appends .avi automa%tically
        obj.writer2 = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input1_video_files(varK1)),'_',char(input2_video_files(varK2)),'_withBbs'));%appends .avi automa%tically
        
        %Input tracks file and output tracks file
        obj.inputTracksFileName = strcat(root_folder,'/special_problem/code_nitk/tracks/working_tracks/',char(input2_video_files(varK2)),'_output_multi_withShifts.mat');
        obj.outputTracksFileName = strcat(root_folder,'/special_problem/code_nitk/tracks/',char(input1_video_files(varK1)),'_',char(input2_video_files(varK2)),'_output.mat');

        % Create two video players, one to display the video,
        % and one to display the foreground mask.
%          obj.maskPlayer = vision.VideoPlayer('Position', [0,0,1920,1080]);
        obj.videoPlayer = vision.VideoPlayer('Position', [0,0,704,480]);

        % Create System objects for foreground detection and blob analysis

        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background.
        
        %NOTE: ForegroundDetector not used
        obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
            'NumTrainingFrames', 300, 'MinimumBackgroundRatio', 0.7, 'LearningRate', 1e-15, 'AdaptLearningRate', true);


        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis System object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteris%tics, such as area, centroid, and the bounding box.

        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 400);
    end
    

    function [mean_track_dist_x,mean_track_dist_y] = findMeanTrackDistance( track, downscaleFactor )
        
        mean_track_dist_x = 0;
        mean_track_dist_y = 0;
        numberOfMovementsInTrack = size(track.all_bboxes_shifted,1);
        for varI_2 = 2:numberOfMovementsInTrack
            mean_track_dist_x = mean_track_dist_x + abs(track.all_bboxes_shifted(varI_2,1)-track.all_bboxes_shifted(varI_2-1,1));
            mean_track_dist_y = mean_track_dist_y + abs(track.all_bboxes_shifted(varI_2,2)-track.all_bboxes_shifted(varI_2-1,2));
        end
        %divide total absolute distance travelled by mosquito by the number
        %of movements made by it
        mean_track_dist_x = int16(mean_track_dist_x/(numberOfMovementsInTrack-1.0));
        mean_track_dist_x = int16(mean_track_dist_x/downscaleFactor);
        mean_track_dist_y = int16(mean_track_dist_y/(numberOfMovementsInTrack-1.0));
        mean_track_dist_y = int16(mean_track_dist_y/downscaleFactor);
    end
    function boolVal = checkBbsOutOfBounds(bbs,imgWidth,imgHeight)
        %check if bbs is lying in the image or not
%         [x, y, width, height]  = bbs ;
        x = bbs(1);
        y = bbs(2);
        width = bbs(3);
        height = bbs(4);
        if(x<1)||(y<1)||((x+width)>=imgWidth)||((y+height)>=imgHeight)
            boolVal = true;
        else
            boolVal = false;
        end
    end
    
    function [frame,mask] = readFrame()
        frame = obj.reader1.step();
        mask = obj.reader2.step();
    end
    
    function [frame2] = readFrame2()
        frame2 = obj.reader3.step();
    end
    
    %Background subtraction and blob analysis
    function [centroids, bboxes] = blobAnalysis(mask)
    
        % Perform blob analysis to find connected components in mask image.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
    end

   
    %Composite downscaled mosquitoes onto target video frames here
    function displaySavedTrackingResults()
%          pause;
        % Convert the frame and the mask to uint8 RGB.
%          frame = im2uint8(frame);
%          mask_RGB = uint8(repmat(mask, [1, 1, 3])) .* 255;
%          mask = uint8(mask).*255;
        
        bboxes = [];
        labels = [];
        
        %create output inpainted image with no bounding boxes of mosquitoes
        image = im2uint8(frame);
        widthOfimage = size(image,2);
        heightOfimage = size(image,1);
        
            
        %Composite 'numOfSplits' number of mosquitoes per frame
        for varI = 1:numel(numOfTrackSets)
            
            currTrackSet = allSavedTracksSplitOutput{varI};
           
            %if a currSet has 0 tracks to start with, dont run it and delete it
            if(trackCounter(varI)>size(currTrackSet,2))
                numOfTrackSets(varI) = [];
                continue;
            end
                        
            
%              mosquito_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/rgb_2/A06I8030_',int2str(allSavedTracksOutput(trackCounter).frameNums(trackFrameNumCounter)),'_',int2str(trackCounter)));
%              mosquito_mask_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/masks_2/A06I8030_',int2str(allSavedTracksOutput(trackCounter).frameNums(trackFrameNumCounter)),'_',int2str(trackCounter)));
            mosquito_img_name = currTrackSet(trackCounter(varI)).img_names{trackFrameCounter(varI)};
            
            mosquito_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/rgb_2/',mosquito_img_name));
            mosquito_mask_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/masks_2/',mosquito_img_name));

            
            mosquito_img_resized = imresize(mosquito_img,1/downScaleFactor);
            mosquito_mask_img_resized = imresize(mosquito_mask_img,1/downScaleFactor);
            
            curr_bbs = currTrackSet(trackCounter(varI)).all_bboxes(trackFrameCounter(varI),:);
            
            curr_bbs(3) = size(mosquito_img_resized,2);
            curr_bbs(4) = size(mosquito_img_resized,1);
            
    %         curr_bbs
            
    %          black_img_resized_NN = imresize(black_img,1/downScaleFactor,'nearest');
    %          black_img_resized_bicubic = imresize(black_img,1/downScaleFactor);
            
            bicubic_pixel_level_threshold = 16;
            image_cropped = image(curr_bbs(2):curr_bbs(2)+curr_bbs(4)-1,curr_bbs(1):curr_bbs(1)+curr_bbs(3)-1,:);
    %              image_cropped(black_img_resized_NN ~= 0) = frame_resized(black_img_resized_NN ~= 0);
    %              black_img_resized_bicubic(black_img_resized_NN>0)

    %              image_cropped(black_img_resized_bicubic >= 16) = frame_resized(black_img_resized_bicubic >= 16);
    %              frame_resized(black_img_resized_bicubic >= 16)

            mosquito_img_resized_double = im2double(mosquito_img_resized);
            image_cropped_double = im2double(image_cropped);
            
            %Option 1 and 2 for compositing
            composition_option = 1;
            
            if(composition_option==1)
                %Option 1 for compositing
                %just multiply all of mosquito pixels with other image...i.e scale final output by intensities
                compositing_condition_0 = (mosquito_mask_img_resized >= bicubic_pixel_level_threshold);
    %             disp('d0')
    %             size(compositing_condition_0)
    %             size(image_cropped_double)
    %             size(mosquito_img_resized_double)   
                image_cropped_double(compositing_condition_0) = image_cropped_double(compositing_condition_0).*mosquito_img_resized_double(compositing_condition_0);

                image_cropped = im2uint8(image_cropped_double);
    %              imshow(mosquito_img_resized_double)
    %              pause
            
            else
            
                %Option 2 for compositing
                %just multiply only mosquito pixels which are whitish, with other image...i.e scale final output by intensities of whitish regions of mosquitoes
                actual_mosquito_intensity_threshold = 64;
                compositing_condition_1 = (mosquito_mask_img_resized >= bicubic_pixel_level_threshold) & (mosquito_img_resized > actual_mosquito_intensity_threshold);
                %just copy only mosquito pixels which are darkish, onto other image...i.e final output will be same as darkish regions of mosquitoes
                compositing_condition_2 = (mosquito_mask_img_resized >= bicubic_pixel_level_threshold) & (mosquito_img_resized <= actual_mosquito_intensity_threshold);% 
                image_cropped_double(compositing_condition_1) = image_cropped_double(compositing_condition_1).*mosquito_img_resized_double(compositing_condition_1);
                image_cropped = im2uint8(image_cropped_double);
                image_cropped(compositing_condition_2) = mosquito_img_resized(compositing_condition_2);
            end
        
            image(curr_bbs(2):curr_bbs(2)+curr_bbs(4)-1,curr_bbs(1):curr_bbs(1)+curr_bbs(3)-1,:) = image_cropped;
            if(enlargeBbsBy2x)
                curr_bbs_x = curr_bbs(1);
                curr_bbs_y = curr_bbs(2);
                curr_bbs_width = curr_bbs(3);
                curr_bbs_height = curr_bbs(4);
                
                curr_bbs_x = curr_bbs_x - curr_bbs_width/2;
                curr_bbs_width = 2*curr_bbs_width;
                curr_bbs_y = curr_bbs_y - curr_bbs_height/2;
                curr_bbs_height = 2*curr_bbs_height;
                if(curr_bbs_x<1)
                curr_bbs_x = 1 ;
                end
                
                if(curr_bbs_y<1)
                curr_bbs_y = 1 ;
                end
                
                if((curr_bbs_x+curr_bbs_width)>=widthOfimage)
                curr_bbs_width = widthOfimage - curr_bbs_x - 1 ;
                end
                
                if((curr_bbs_y+curr_bbs_height)>=heightOfimage)
                curr_bbs_height = heightOfimage - curr_bbs_y - 1 ;
                end
                
                curr_bbs(1) = curr_bbs_x;
                curr_bbs(2) = curr_bbs_y; 
                curr_bbs(3) = curr_bbs_width;
                curr_bbs(4) = curr_bbs_height ;
                
            end
%              if(showMergedTrackNums)
            curr_track_id = strcat(num2str(varI),'.',num2str(trackCounter(varI)));
            image = insertObjectAnnotation(image, 'rectangle', ...
                                curr_bbs, (curr_track_id) );
%              else
%                  image = insertObjectAnnotation(image, 'rectangle', ...
%                                      curr_bbs, int2str(trackCounter) );
%              end
            
            
            trackFrameCounter(varI) = trackFrameCounter(varI) + 1;
            
            %if the frames of a track of currSet is over, switch to next track in set
            if(trackFrameCounter(varI)>currTrackSet(trackCounter(varI)).trackLength)
                trackFrameCounter(varI) = 1;
                trackCounter(varI) = trackCounter(varI) + 1;
                %if end of currSet is reached and all its tracks are completed, remove it
                if(trackCounter(varI)>size(currTrackSet,2))
%                      disp('d0')
                    numOfTrackSets(varI) = [];
%                      trackCounter(varI)
%                      trackFrameCounter(varI)
%                      currTrackSet
%                      size(currTrackSet(trackCounter(varI)),2)
                    
                end
%                  currTrackSet(trackCounter(varI)).trackLength
            end
            
            
        end

%          Display the mask and the frame.
        obj.videoPlayer.step(image);
%          pause;
%          pause(0.03)
%          obj.videoPlayer.step(image);
%          obj.maskPlayer.step(image_withBbs);
%          obj.videoPlayer.step(black_img_resized_bicubic);
        
%          save video of mask (grayscale) and frame to file
        writeVideo(obj.writer1,image);
%          writeVideo(obj.writer2,image_withBbs);

    end
   
end