%function to display downsized mosquitoe tracks on videos frames of a given dataset. This code uses joining of multiple tracks, to increase the area of tracks.
%This composites only a single mosquito onto the video per frame 

function MotionBasedMultiObjectTrackingExample()

%setting seed to get reproducible results for every run of this function
seed = 999;
rng(seed)

enlargeBbsBy2x = true;
showMergedTrackNums = false;

input1_video_files = [string('PRG1')];%,string('PRG6'),string('PRG7'),string('PRG14'),string('PRG22'),string('PRG23'),string('PRG28'),string('PRG29')]; 
input2_video_files = [string('A06I8030')];%,string('A06I8031'),string('A06I8032'),string('A06I9821'),string('A06I9822'),string('A06I9823'),string('A06I9824')];

enableRestartFromCenterFlag = false;
enableSmoothAnglesBWSubtracks = true;%false;%true;
enableDelayedTrackRestartFlag = true;

enableSizeMatchingBetweenTracks = true;

enableAngleMatchingBetweenTracksFlag = true;

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

        trackStartIndices = [];
        for varI = 1:numOfTracks
            %startFrameNum,endFrameNum,trackId,trackIndex
            trackStartIndices = cat(1, trackStartIndices,[1,1,1,]);% [allSavedTracks(varI).frameNums(1), allSavedTracks(varI).frameNums(end), allSavedTracks(varI).id, varI] );
        end
        trackStartIndices = sortrows(trackStartIndices);
        trackStartIndices;
        nextId = 1; % ID of the next track
        
        %after combining the tracks,give each continuous track a number or id, and when the track breaks off and when a new track is started at a random edge, give that the next number as id.
        [allSavedTracksOutput(:).trackId] = deal(1);
        [allSavedTracksOutput(:).downScaleFactor] = deal(1);
        [allSavedTracksOutput(:).all_bboxes_shifted_output] = deal(zeros(1,4));
        [allSavedTracksOutput(:).startrow] = deal(1);
        [allSavedTracksOutput(:).startcol] = deal(1);
        
        %Positions of where mosquito frames need to be placed
        startrows = [100]; %[700,700,100,100,300];
        startcols = [50]; %[1300,420,1300,1100,400];

        %reduce mosquitoes size by this factor:
        downScaleFactors = [25.0];%[5.0]; %[5.0,10.0,20.0,22.0,15.0];
        downScaleFactorsOriginal = downScaleFactors;     
        mean_track_distance_x = 0;
        mean_track_distance_y = 0;
        
        %for /home/nitin/gatech_courses/special_problem/code_nitk/videos/input/camnet_dataset/Videos/PRG1.avi
        imgWidth = 704;
        imgHeight = 480;
        
        %for /home/nitin/gatech_courses/special_problem/lasiesta_database/I_BS_01/I_BS_01/I_BS_01-1.bmp
%          imgWidth = 352;
%          imgHeight = 288;
        
        %find angles of mosquito tracks for entire trackset:
        numberOfMosquitoesInAllTracks = sum([allSavedTracksOutput.totalVisibleCount]);
        anglesBetweenMosquitoesInAllTracks = zeros(1,numberOfMosquitoesInAllTracks-1);
        trackIdsBetweenMosquitoesInAllTracks = zeros(1,numberOfMosquitoesInAllTracks-1);
        counter = 1;
        
        numOfTracks = size(allSavedTracksOutput,2);
        %angle at which track starts to move
        startAnglesOfTracks = zeros(1,numOfTracks);
        %angle at which track moves just before it ends
        endAnglesOfTracks = zeros(1,numOfTracks);
        
        p2 = [0,0];
        p1 = [0,0];
        
        %dummy code
%          tmp = allSavedTracksOutput(1);
%          allSavedTracksOutput(1)= allSavedTracksOutput(278);
%          allSavedTracksOutput(278) =tmp;
        
        %Calculate angles between frames of tracks in default order
        [anglesBetweenMosquitoesInAllTracks,trackIdsBetweenMosquitoesInAllTracks,startAnglesOfTracks,endAnglesOfTracks] = findAnglesBetweenTracks(allSavedTracksOutput,imgHeight,anglesBetweenMosquitoesInAllTracks,trackIdsBetweenMosquitoesInAllTracks,startAnglesOfTracks,endAnglesOfTracks);
        
        save('startAnglesOfTracks','startAnglesOfTracks');
        save('endAnglesOfTracks','endAnglesOfTracks');
        save('anglesBetweenMosquitoesInAllTracks','anglesBetweenMosquitoesInAllTracks');
        save('trackIdsBetweenMosquitoesInAllTracks','trackIdsBetweenMosquitoesInAllTracks');
        
        if(enableAngleMatchingBetweenTracksFlag)
        
            %Match tracks whose exit angles match entry angles of next track
            [allSavedTracksOutput,startAnglesOfTracks,endAnglesOfTracks] = matchAnglesBetweenTracks(allSavedTracksOutput,anglesBetweenMosquitoesInAllTracks,trackIdsBetweenMosquitoesInAllTracks,startAnglesOfTracks,endAnglesOfTracks);
            
            %Recalculate angles between frames, with matched tracks
            [anglesBetweenMosquitoesInAllTracks,trackIdsBetweenMosquitoesInAllTracks,startAnglesOfTracks,endAnglesOfTracks] =findAnglesBetweenTracks(allSavedTracksOutput,imgHeight,anglesBetweenMosquitoesInAllTracks,trackIdsBetweenMosquitoesInAllTracks,startAnglesOfTracks,endAnglesOfTracks);
            
        end


        save('startAnglesOfTracks2','startAnglesOfTracks');
        save('endAnglesOfTracks2','endAnglesOfTracks');
        save('anglesBetweenMosquitoesInAllTracks2','anglesBetweenMosquitoesInAllTracks');
        save('trackIdsBetweenMosquitoesInAllTracks2','trackIdsBetweenMosquitoesInAllTracks');
        save('tmpMat','allSavedTracksOutput');
%          return;

        %code to find average speed per subtrack
        avgSpeedOfTracks = zeros(1,size(allSavedTracksOutput,2));
%          avgSpeedOfTracks = calculateAvgSpeedOfSubTracks(allSavedTracksOutput,avgSpeedOfTracks);
        
        curr_counter = 1;
        for varI = 1:size(allSavedTracksOutput,2)
            allSavedTracksOutput(varI).all_bboxes_shifted_output = zeros(allSavedTracksOutput(varI).totalVisibleCount,4);
            allSavedTracksOutput(varI).trackId = zeros(allSavedTracksOutput(varI).totalVisibleCount,1);
            if(varI>1)
               
            end
            for varJ = 1:(allSavedTracksOutput(varI).totalVisibleCount)
                
                allSavedTracksOutput(varI).trackId(varJ) = curr_counter;
                %if first position of first track only,set to user defined startrow and
                %start col
                if(varJ==1&&varI==1)
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1:2) = [startcols(1),startrows(1)];
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactors(1));
                elseif(varJ==1&&varI>=1)
                %if first position of tracks after the first track,set to
                %previous track's last position + or - the mean track distance of
                %previous track (both in x direction and y direction), to enable smooth continuous tracks
                
                    prev_point = [allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) , imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2)];
                    prev_to_prev_point = [allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end-1,1), imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end-1,2)];
                    
                    angle = calculate_angle(prev_point,prev_to_prev_point);
                    
                    %for debugging purpose, if we didnt smooth out the angles*********************
%                      curr_point_orig_scale = [allSavedTracksOutput(varI).all_bboxes_shifted(1,1) , imgHeight-allSavedTracksOutput(varI).all_bboxes_shifted(1,2)];
%                      prev_point_orig_scale = [allSavedTracksOutput(varI-1).all_bboxes_shifted(end,1) , imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted(end,2)];
%                      angle = calculate_angle(curr_point_orig_scale,prev_point_orig_scale)
%                      prev_point_orig_scale
%                      curr_point_orig_scale
%  %                      allSavedTracksOutput(varI)
%                      pause
                    %*********************for debugging purpose, if we didnt smooth out the angles
                    
%                      prev_point
%                      prev_to_prev_point
                    
                    cos_theta = cos(angle*3.141592653589/180);
                    sin_theta = sin(angle*3.141592653589/180);
                    
                    
                    mean_track_dist_x_n_y = (mean_track_distance_x+mean_track_distance_y)/2.0;
                    
                    x_sign = 1;
                    y_sign = -1;
                    if(angle>90)
                        x_sign = 1;
                    elseif(angle<-90&&angle>=-180)
                        x_sign = 1;
                        y_sign = -1;
                    elseif(angle<-90&&angle>=-180)
                        y_sign = 1;
                    end
                    
%                      disp('d1')
%                      disp(cos_theta)
%                      disp(cos_theta*x_sign);
%                      disp(sin_theta)
%                      disp(sin_theta*y_sign);
%                      disp(prev_to_prev_point);
%                      disp(prev_point);
%                      disp(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1));
%                      disp(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2));
%                      
%                      disp(mean_track_dist_x_n_y*cos_theta*x_sign);
%                      disp(mean_track_dist_x_n_y*sin_theta*y_sign);
%                      
%                      disp(int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_dist_x_n_y*cos_theta*x_sign));
%                      disp(int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_dist_x_n_y*sin_theta*y_sign));
%                      y_sign = -1*y_sign;
                    if(enableSmoothAnglesBWSubtracks)
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_dist_x_n_y*cos_theta*x_sign);
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_dist_x_n_y*sin_theta*y_sign);
                    else
                        signOfJump  = [-1,1];
%                          choose between -1, and 1 randomly
%                          signOfJump( randi(length(signOfJump)) );
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_distance_x*signOfJump( randi(length(signOfJump)) ));
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_distance_y*signOfJump( randi(length(signOfJump)) ));
                    end
                    
%                      curr_point = [allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1),imgHeight-allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2)]
%                      angle
%                      angle2 = calculate_angle(curr_point,prev_point)
%                      pause
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(1,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(1,3:4)/downScaleFactors(1));
                else
%                      varJ
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,1)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,1))/downScaleFactors(1)+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,1));
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,2)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,2))/downScaleFactors(1)+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,2));
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactors(1));
                end
                
%                  imgWidth = 704;
%                  imgHeight = 480;
                %if the current position of mosquito is outside of the
                %video region, randomly continue it at another edge, as
                %though a new mosquito entered into the video
%                 disp('d0');
                curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:);
%                 checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)
                if(checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)==true)
                    curr_counter = curr_counter + 1;
                    allSavedTracksOutput(varI).trackId(varJ) = curr_counter;
                    %these distances are the mean distance the mosquito moved in x
                    %and y direction at one instant. They will be used to jump from
                    %one track to another track smoothly.
                    [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance(allSavedTracksOutput(varI),downScaleFactors(1));
                    %decide which side the mosquito enters from,
                    %1-left,2-right,3-top or 4-bottom
%                     mean_track_distance_x
%                     mean_track_distance_y
%                     pause
                    rand_edge = randi(4);
                    %add this value to x or y position of reentry to
                    %restart the track a bit further away from edges to prevent
                    %frequent restarting of tracks, via frequenting
                    %exiting.
                    shift_random_entry = 20;

                    %hardcoding jump distances when new long track starts, to small jumps
%                      mean_track_distance_x = 1;
%                      mean_track_distance_y = 1;

                    if(rand_edge==1)
                        rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = 1 + mean_track_distance_x + shift_random_entry;
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
                    elseif(rand_edge==2)
                        rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = imgWidth - mean_track_distance_x - curr_bbs(3) - shift_random_entry ;
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
                    elseif(rand_edge==3)
                        rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = 1 + mean_track_distance_y + shift_random_entry;   
                    else
                        rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = imgHeight - mean_track_distance_y - curr_bbs(4) - shift_random_entry;
                    end
                    
                    if(enableRestartFromCenterFlag)
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = startcols(1);
                        allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = startrows(1);
                    end
                end
%                 curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:)
%                 disp('d1');
                
                
            end
            %these distances are the mean distance the mosquito moved in x
            %and y direction at one instant. They will be used to jump from
            %one track to another track smoothly.
            [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance(allSavedTracksOutput(varI),downScaleFactors(1));
            
%              allSavedTracksOutput(varI).all_bboxes_shifted_output
        end
%          allSavedTracksOutput(1).all_bboxes_shifted_output
%          allSavedTracksOutput(1).all_bboxes_shifted

        frameNum=0;
        numOfFramesInVideo = 20000000;%number of video frames in folder
        runNFrames = numOfFramesInVideo;
        runTillEndFlag = true;
        skipNFrame = 0;

        

        downScaleFactor = downScaleFactors(1);
        origDownScaleFactor = downScaleFactor;

        %Random shift value for ROI box in pixels
        rand_shift_val = 5;

        %write video to file
        open(obj.writer1)
        open(obj.writer2)

        activeTrackIdNFrameNum = [];

        startTrackCounter = 1;
        endTrackCounter = 1;
        %track number of each long continuous track which is made of subtracks
        trackCounter = 1;
        %subtrackNumber of subtracks which are joined
        subTrackCounter = 0;
        %subtrackFrameNumber of subtracks which are joined
        subTrackFrameCounter = 0;
        prevTrackFrameNumCounter = 1;
        subTrackFrameNumCounter = 0;
        numOfWhitePixelsInMosqBbs = 0;
        numOfWhitePixelsInMosqBbsInPreviousTrack = 0;
        matchMosquitoSizeFlag = false;
        startOfCurrentSubTrack = false;
        % Detect moving objects, and track them across video frames.
        
        sizeOfPixelSizesArray = 10000;
        pixelSizesArray = ones(1,sizeOfPixelSizesArray)*-1;  
        trackIdOfPixelSizesArray = ones(1,sizeOfPixelSizesArray)*-1;
        pixelSizesArrayCounter = 1;
        
        sizeOfAnglesArray = sizeOfPixelSizesArray;
        anglesArray = ones(1,sizeOfAnglesArray)*-1;  
        trackIdOfAnglesArray = ones(1,sizeOfAnglesArray)*-1;
        anglesArrayCounter = 1;
        
        pointsForAnglesArray = ones(2*sizeOfAnglesArray+2,2*sizeOfAnglesArray+2)*-1; 
        pointsForEndOfSubTracksArray = ones(2*sizeOfAnglesArray+2,3)*-1;  

%          1 2
%          x1,y1,x2,y2,x3,y3
        delayLowerLimit = 60;%30
        delayUpperLimit = 180;%120
        
        %delay between entry of new mosquitoes into frame
        mosquitoEntryDelay = randi([delayLowerLimit,delayUpperLimit],1,1);
        mosquitoEntryDelayFlag = true;
        
        interFrameDisplayDelay = 0.03;%0.00001 ;%in seconds
        
        %if you want to step through video instead of running through frames continuously
        stepFlag = false;
        
        %if you want to run the processing without showing video, then set this to false
        displayImageFlag = false;
        
        %if you want to draw blue line and green circles on tracks, with green circle signifying the connection between subtracks
        drawTrackPathFlag = false;
        
        if(~enableDelayedTrackRestartFlag)
            delayLowerLimit = 0;
            delayUpperLimit = 0;
            interFrameDisplayDelay = 0.00001 ;%in seconds
        end
        
        prev_point = [200,imgHeight-200];%as Y co-ordinate at top point in image is 0
        curr_point = [200,imgHeight-200];%as Y co-ordinate at top point in image is 0
        
        line_color = 'red';
        
        indexPtr = 1;
        indexPtr2 = 1;
        
        %if you want to read in a set of images onto which you want to composite the mosquitoes
        imgFolder = '/home/nitin/gatech_courses/special_problem/lasiesta_database/I_BS_01/I_BS_01/';
        imgRootName = 'I_BS_01-';
        imgEndName = '.bmp';
        listOfImgs=dir([imgFolder '/*.bmp'])
        numOfImgs=size(listOfImgs,1)
        imgCounter=1;
        imgMaskFolder = '/home/nitin/gatech_courses/special_problem/lasiesta_database/I_BS_01/I_BS_01-GT/';
        imgMaskRootName = 'I_BS_01-GT_';
        
        personPresentInCurrFrameFlag=0;
        personPresentInPrevFrameFlag=0;
        
        noOcclFlag = 0;
        prevNoOcclFlag = noOcclFlag;
        
        occlCountDownTimer = 0;
        
        while ~isDone(obj.reader1) | ~isDone(obj.reader2) | ~isDone(obj.reader3)

        %      disp('d1')
            %tic
            %mosquito frame and its corresponding background mask
            [frame,mask] = readFrame();
            mask = rgb2gray(mask);
            mask = logical(mask);
            %toc
        %      size(mask)
        %      disp('d2')
            %tic
            disp('frameNum')
            disp(frameNum)
            frameNum = frameNum + 1;
            
            if(frameNum==runNFrames&&runTillEndFlag==false)
                break
            end

            if(frameNum<=skipNFrame)
                continue
            end
            
            if(frameNum==2)
                pause
            end

            
            %if using input in the form of video
            frame2 = readFrame2();
            
            %use white image instead of scene video:
%              frame2 = ones(size(frame2,1),size(frame2,2),3)*255;
            
            %read from image set instead of scene video:
%              [frame2,frame2Mask] = readImage();
%              frame2 = frame2Mask;
%              frame2Mask = rgb2gray(frame2Mask);
%              frame2Mask = im2double(frame2Mask);
%              frame2Mask = im2bw(frame2Mask, 0.25);%0.25 is the binary threshold
            imgCounter = imgCounter + 1;

            %tic
        %      [centroids, bboxes, mask] = detectObjects(frame);
            [centroids, bboxes] = blobAnalysis(mask);
            %toc
            %display tracks on frame in gui here:
            
        %      disp('d4')
            
            if(subTrackFrameCounter==0)
                subTrackCounter = subTrackCounter + 1;
                matchMosquitoSizeFlag = true;
                startOfCurrentSubTrack = true;
                subTrackFrameCounter = allSavedTracksOutput(subTrackCounter).totalVisibleCount; 
                
                %if you want to use only average of 1st track for entire video, to match sizes between tracks
%                  if(subTrackCounter==1)
%                      prevTrackFrameNumCounter = allSavedTracksOutput(subTrackCounter).totalVisibleCount; 
%                  end
                
                if(subTrackCounter>1)
                    %get previous track's size or number of frames 
                    prevTrackFrameNumCounter = allSavedTracksOutput(subTrackCounter-1).totalVisibleCount; 
                end
                subTrackFrameNumCounter = 1;
                if(subTrackCounter>size(allSavedTracksOutput,2))
                    disp('All tracks complete');
                    break;
                end
            end
%              allSavedTracksOutput(subTrackCounter)
%              return
            %tic
            
            %when one long sequence of joined track ends and a new one starts elsewhere, add delay to entrance of new track
            if(allSavedTracksOutput(subTrackCounter).trackId(subTrackFrameNumCounter)>trackCounter)
                trackCounter = allSavedTracksOutput(subTrackCounter).trackId(subTrackFrameNumCounter);
                mosquitoEntryDelayFlag = true;
                %use original downscale factor when new long track starts
%                  downScaleFactor = origDownScaleFactor;
                matchMosquitoSizeFlag = true;
                mosquitoEntryDelay = randi([delayLowerLimit,delayUpperLimit],1,1);
            end
            
            if(mosquitoEntryDelayFlag)
                while(mosquitoEntryDelay>0)
                    mosquitoEntryDelay = mosquitoEntryDelay - 1 ;
                    fprintf('delay %i\n',mosquitoEntryDelay)
                
                    %display frame without compositing mosquitoes
                    image = im2uint8(frame2);
                    displayImage(image,interFrameDisplayDelay,stepFlag,displayImageFlag);
                    continue
                end
            end
            
%              downScaleFactor
            
            compositionFunction();
            
            %This portion of code may be commented
            %%{
            %return from nested call, to terminal
%              if(pixelSizesArrayCounter>size(pixelSizesArray,2))
%                  return
%              end
            %%}
            
            
            %return from nested call, to terminal
%              if(anglesArrayCounter>size(anglesArray,2))
%                  return
%              end
            
            matchMosquitoSizeFlag = false;
            startOfCurrentSubTrack = false;
            
            mosquitoEntryDelayFlag = false;
            
            %toc
            subTrackFrameCounter = subTrackFrameCounter-1;
            subTrackFrameNumCounter = subTrackFrameNumCounter + 1;
            %5,4,3,2,1,0
            %1,2,3,4,5,6
        
        %      disp('d5')
            %tic
            idsToBeRemoved = [];
            for varI = 1:size(activeTrackIdNFrameNum,1)
                activeTrackIdNFrameNum(varI,2) = activeTrackIdNFrameNum(varI,2) + 1;
                if(activeTrackIdNFrameNum(varI,2)>activeTrackIdNFrameNum(varI,4))
                    idsToBeRemoved = [idsToBeRemoved,varI];
                end
            end
            activeTrackIdNFrameNum(idsToBeRemoved,:)=[];
            activeTrackIdNFrameNum;
            %toc
            
%              %if you are reading from a set of images, on which mosquitoes are composited onto.
%              if(imgCounter>numOfImgs)
%                  break;
%              end
            
            if(imgCounter==100)
                imgCounter = 1;
            end
            
        end

        %Save any remaining tracks still active at end of video
        %  saveValidActiveTracksBeforeClose()

        %save resized tracksMat to file
        save(obj.outputTracksFileName,'allSavedTracksOutput');


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
        obj.writer1 = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input1_video_files(varK1)),'_',char(input2_video_files(varK2)),'_noBbsXXX1'));%appends .avi automa%tically
        obj.writer2 = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input1_video_files(varK1)),'_',char(input2_video_files(varK2)),'_noBbsXXX2'));%appends .avi automa%tically
        
        %Input tracks file and output tracks file
        obj.inputTracksFileName = strcat(root_folder,'/special_problem/code_nitk/tracks/working_tracks/','allTracksAllVids.mat');
%          obj.inputTracksFileName = strcat(root_folder,'/special_problem/code_nitk/tracks/working_tracks/',char(input2_video_files(varK2)),'_output_multi_withShifts.mat');
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
    
%      function [avgSpeedOfTracks] = calculateAvgSpeedOfSubTracks(allSavedTracksOutput,avgSpeedOfTracks)
%          
%          for varI = 1:size(allSavedTracksOutput,2)
%              for varJ = 1:(allSavedTracksOutput(varI).totalVisibleCount)
%                  if(varJ>1)
%                      prev_point = allSavedTracksOutput(varI).
%                  end            
%              end 
%          end
%      
%      end

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
    
    
    
    
    function [mean_track_dist_x,mean_track_dist_y] = findMeanTrackDistance2( track )
        
        mean_track_dist_x = 0;
        mean_track_dist_y = 0;
        numberOfMovementsInTrack = size(track.all_bboxes_shifted_output,1);
        for varI_2 = 2:numberOfMovementsInTrack
            mean_track_dist_x = mean_track_dist_x + abs(track.all_bboxes_shifted_output(varI_2,1)-track.all_bboxes_shifted_output(varI_2-1,1));
            mean_track_dist_y = mean_track_dist_y + abs(track.all_bboxes_shifted_output(varI_2,2)-track.all_bboxes_shifted_output(varI_2-1,2));
        end
        %divide total absolute distance travelled by mosquito by the number
        %of movements made by it
        mean_track_dist_x = int16(mean_track_dist_x/(numberOfMovementsInTrack-1.0));
%          mean_track_dist_x = int16(mean_track_dist_x/downscaleFactor);
        mean_track_dist_y = int16(mean_track_dist_y/(numberOfMovementsInTrack-1.0));
%          mean_track_dist_y = int16(mean_track_dist_y/downscaleFactor);
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
    
    function [frame2,frame2Mask] = readImage()
        frame2 = imread(strcat(imgFolder,imgRootName,int2str(imgCounter),'.bmp'));
        frame2Mask = imread(strcat(imgMaskFolder,imgMaskRootName,int2str(imgCounter),'.png'));
    end
    
    %Background subtraction and blob analysis
    function [centroids, bboxes] = blobAnalysis(mask)
    
        % Perform blob analysis to find connected components in mask image.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
    end
    
    function displayImage(image,delay,stepFlag,displayImageFlag)
        if(displayImageFlag)
            obj.videoPlayer.step(image);
            if(stepFlag)
                pause
            else
                pause(delay)
            end
        end
    end
    
    function [allSavedTracksOutput] = rescaleBbs(allSavedTracksOutput,subTrackCounter,downScaleFactor,trackCounter)
        
        varI = subTrackCounter;

        %these distances are the mean distance the mosquito moved in x
        %and y direction at one instant. They will be used to jump from
        %one track to another track smoothly.
        [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance2(allSavedTracksOutput(varI-1));
        
         curr_counter = trackCounter;

        for varJ = 1:(allSavedTracksOutput(varI).totalVisibleCount)
            
            %if first position of first track only,set to user defined startrow and
            %start col
            if(varJ==1&&varI==1)
                allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1:2) = [startcols(1),startrows(1)];
                allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactor);
            elseif(varJ==1&&varI>=1)
            %if first position of tracks after the first track,set to
            %previous track's last position + or - the mean track distance of
            %previous track (both in x direction and y direction), to enable smooth continuous tracks
            
                prev_point = [allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) , imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2)];
                prev_to_prev_point = [allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end-1,1), imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end-1,2)];
                
                angle = calculate_angle(prev_point,prev_to_prev_point);
                
                %for debugging purpose, if we didnt smooth out the angles*********************
%                      curr_point_orig_scale = [allSavedTracksOutput(varI).all_bboxes_shifted(1,1) , imgHeight-allSavedTracksOutput(varI).all_bboxes_shifted(1,2)];
%                      prev_point_orig_scale = [allSavedTracksOutput(varI-1).all_bboxes_shifted(end,1) , imgHeight-allSavedTracksOutput(varI-1).all_bboxes_shifted(end,2)];
%                      angle = calculate_angle(curr_point_orig_scale,prev_point_orig_scale)
%                      prev_point_orig_scale
%                      curr_point_orig_scale
%  %                      allSavedTracksOutput(varI)
%                      pause
                %*********************for debugging purpose, if we didnt smooth out the angles
                
%                      prev_point
%                      prev_to_prev_point
                
                cos_theta = cos(angle*3.141592653589/180);
                sin_theta = sin(angle*3.141592653589/180);
                
                
                mean_track_dist_x_n_y = (mean_track_distance_x+mean_track_distance_y)/2.0;
                
                x_sign = 1;
                y_sign = -1;
                if(angle>90)
                    x_sign = 1;
                elseif(angle<-90&&angle>=-180)
                    x_sign = 1;
                    y_sign = -1;
                elseif(angle<-90&&angle>=-180)
                    y_sign = 1;
                end
                
%                      disp('d1')
%                      disp(cos_theta)
%                      disp(cos_theta*x_sign);
%                      disp(sin_theta)
%                      disp(sin_theta*y_sign);
%                      disp(prev_to_prev_point);
%                      disp(prev_point);
%                      disp(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1));
%                      disp(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2));
%                      
%                      disp(mean_track_dist_x_n_y*cos_theta*x_sign);
%                      disp(mean_track_dist_x_n_y*sin_theta*y_sign);
%                      
%                      disp(int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_dist_x_n_y*cos_theta*x_sign));
%                      disp(int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_dist_x_n_y*sin_theta*y_sign));
%                      y_sign = -1*y_sign;
                if(enableSmoothAnglesBWSubtracks)
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_dist_x_n_y*cos_theta*x_sign);
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_dist_x_n_y*sin_theta*y_sign);
                else
                    signOfJump  = [-1,1];
%                          choose between -1, and 1 randomly
%                          signOfJump( randi(length(signOfJump)) );
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,1) + mean_track_distance_x*signOfJump( randi(length(signOfJump)) ));
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2) = int16(allSavedTracksOutput(varI-1).all_bboxes_shifted_output(end,2) + mean_track_distance_y*signOfJump( randi(length(signOfJump)) ));
                end
                
%                      curr_point = [allSavedTracksOutput(varI).all_bboxes_shifted_output(1,1),imgHeight-allSavedTracksOutput(varI).all_bboxes_shifted_output(1,2)]
%                      angle
%                      angle2 = calculate_angle(curr_point,prev_point)
%                      pause
                allSavedTracksOutput(varI).all_bboxes_shifted_output(1,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(1,3:4)/downScaleFactor);
            else
%                      varJ
                allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,1)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,1))/downScaleFactor+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,1));
                allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = int16((allSavedTracksOutput(varI).all_bboxes_shifted(varJ,2)-allSavedTracksOutput(varI).all_bboxes_shifted(varJ-1,2))/downScaleFactor+allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ-1,2));
                allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,3:4) = int16(allSavedTracksOutput(varI).all_bboxes_shifted(varJ,3:4)/downScaleFactor);
            end
            
%                  imgWidth = 704;
%                  imgHeight = 480;
            %if the current position of mosquito is outside of the
            %video region, randomly continue it at another edge, as
            %though a new mosquito entered into the video
%                 disp('d0');
            curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:);
%                 checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)
            if(checkBbsOutOfBounds(curr_bbs,imgWidth,imgHeight)==true)
                curr_counter = curr_counter + 1;
                allSavedTracksOutput(varI).trackId(varJ) = curr_counter;
                %these distances are the mean distance the mosquito moved in x
                %and y direction at one instant. They will be used to jump from
                %one track to another track smoothly.
                [mean_track_distance_x,mean_track_distance_y] = findMeanTrackDistance(allSavedTracksOutput(varI),downScaleFactor);
                %decide which side the mosquito enters from,
                %1-left,2-right,3-top or 4-bottom
%                     mean_track_distance_x
%                     mean_track_distance_y
%                     pause
                rand_edge = randi(4);
                %add this value to x or y position of reentry to
                %restart the track a bit further away from edges to prevent
                %frequent restarting of tracks, via frequenting
                %exiting.

                shift_random_entry = 20;%20

                %hardcoding jump distances when new long track starts, to small jumps
%                  mean_track_distance_x = 1;
%                  mean_track_distance_y = 1;

                if(rand_edge==1)
                    rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = 1 + mean_track_distance_x + shift_random_entry;
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
                elseif(rand_edge==2)
                    rand_y_pos = randi(imgHeight-curr_bbs(4)-1);
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = imgWidth - mean_track_distance_x - curr_bbs(3) - shift_random_entry ;
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = rand_y_pos;
                elseif(rand_edge==3)
                    rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = 1 + mean_track_distance_y + shift_random_entry;   
                else
                    rand_x_pos = randi(imgWidth-curr_bbs(3)-1);          
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = rand_x_pos ;
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = imgHeight - mean_track_distance_y - curr_bbs(4) - shift_random_entry;
                end

                if(enableRestartFromCenterFlag)
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,1) = startcols(1);
                    allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,2) = startrows(1);
                end
            end
%                 curr_bbs = allSavedTracksOutput(varI).all_bboxes_shifted_output(varJ,:)
%                 disp('d1');
            
            
        end
       
        
%              allSavedTracksOutput(varI).all_bboxes_shifted_output
    
    end

    
    %Composite downscaled mosquitoes onto target video frames here
    function compositionFunction()
%          pause;
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask_RGB = uint8(repmat(mask, [1, 1, 3])) .* 255;
%          mask = uint8(mask).*255;
        
        bboxes = [];
        labels = [];
        
        %create output inpainted image with no bounding boxes of mosquitoes
        image = im2uint8(frame2);
        widthOfimage = size(image,2);
        heightOfimage = size(image,1);
        
        bicubic_pixel_level_threshold = 64;
%          allSavedTracksOutput(subTrackCounter).id;
        
        
        %This will not work when tracks in allSavedTracksOutput are mixed up
%          mosquito_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/rgb_2/A06I8030_',int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(subTrackCounter)));
%          mosquito_mask_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/masks_2/A06I8030_',int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(subTrackCounter)));
        
        %This will work when tracks in allSavedTracksOutput are mixed up
%          mosquito_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/rgb_2/A06I8030_',int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(allSavedTracksOutput(subTrackCounter).id)));
%          mosquito_mask_img = imread(strcat('/home/nitin/gatech_courses/special_problem/code_nitk/mosquito_bbs/A06I8030/masks_2/A06I8030_',int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(allSavedTracksOutput(subTrackCounter).id)));

        %This will work when tracks in allSavedTracksOutput are mixed up
        mosquito_img = imread(strcat(allSavedTracksOutput(subTrackCounter).rgb_bbs_path,int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(allSavedTracksOutput(subTrackCounter).id)));
        mosquito_mask_img = imread(strcat(allSavedTracksOutput(subTrackCounter).mask_bbs_path,int2str(allSavedTracksOutput(subTrackCounter).frameNums(subTrackFrameNumCounter)),'_',int2str(allSavedTracksOutput(subTrackCounter).id)));

        diffInPixelsThresh = 500;
        
%          avgNumOfWhitePixelsInMosqBbs = 138;% numOfWhitePixelsInMosqBbs;

        


%          disp('numOfWhitePixelsInMosqBbsInPreviousTrack')
%          disp(numOfWhitePixelsInMosqBbsInPreviousTrack)
%          disp('prevTrackFrameNumCounter')
%          disp(prevTrackFrameNumCounter)
        
        %use average mosquito size of previous track for matching size of next track
        if(startOfCurrentSubTrack&&frameNum>1)
            numOfWhitePixelsInMosqBbsInPreviousTrack/prevTrackFrameNumCounter;
            numOfWhitePixelsInMosqBbsInPreviousTrack;
            prevTrackFrameNumCounter;
            allSavedTracksOutput(subTrackCounter-1).totalVisibleCount;
    %          pause
        end
        
        %use mosquito size of end of previous track for next track
%          avgNumOfWhitePixelsInMosqBbs = numOfWhitePixelsInMosqBbs;
        
        %use average size of previous track for next track
        %avgNumOfWhitePixelsInMosqBbs = numOfWhitePixelsInMosqBbsInPreviousTrack/prevTrackFrameNumCounter
        
        %use a value between average size of previous track and mosquito size of end of previous track , for next track
        weightBwAvgAndLastTrackSize = 0.5;
        avgNumOfWhitePixelsInMosqBbs = numOfWhitePixelsInMosqBbsInPreviousTrack/prevTrackFrameNumCounter;
        avgNumOfWhitePixelsInMosqBbs = numOfWhitePixelsInMosqBbs + weightBwAvgAndLastTrackSize*(avgNumOfWhitePixelsInMosqBbs-numOfWhitePixelsInMosqBbs);
        


%          disp('subTrackCounter')
%          disp(subTrackCounter)
        
        deltaDownScaleFactor = 0.1;
        deltaDownScaleFactorConst = deltaDownScaleFactor;
        %Code Commented out below
        %%{
        %To prevent oscillation between 2 downScaleFactors
        tryNtimes = 500;
        tryCount = tryNtimes;
        diffInPixels = 0;
        const1 = 2.0;
        const2 = 4.0;
        const3 = 4.0;
        
        function [diffInPixels] = diffInPixelsFun(downScaleFactor)
%          downScaleFactor
                mosquito_mask_img_resized = imresize(mosquito_mask_img,1/downScaleFactor);
                currNumOfWhitePixelsInMosqBbs =  nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
                diffInPixels = currNumOfWhitePixelsInMosqBbs-avgNumOfWhitePixelsInMosqBbs ;
                diffInPixels = abs(diffInPixels);
        end
        if(frameNum>1&&startOfCurrentSubTrack&&tryCount>0)
            minBnd = downScaleFactors(1)-2;
            if(minBnd<1)
                minBnd = 1;
            end
            maxBnd = downScaleFactors(1)+2;
            downScaleFactor = fminbnd(@diffInPixelsFun,minBnd,maxBnd);
            diffInPixelsFun(downScaleFactor);
%              pause

        end
        %{
        while(frameNum>1&&startOfCurrentSubTrack&&tryCount>0)
            
            %To prevent oscillation between 2 downScaleFactors
            if(mod(tryCount,50)==0)
                deltaDownScaleFactor = deltaDownScaleFactor*5;
            end
            mosquito_mask_img_resized = imresize(mosquito_mask_img,1/downScaleFactor);
            currNumOfWhitePixelsInMosqBbs =  nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
            
            
            diffInPixels = currNumOfWhitePixelsInMosqBbs-avgNumOfWhitePixelsInMosqBbs ;
            downScaleFactor
            diffInPixels = abs(diffInPixels);
            tryCount = tryCount-1
%              downScaleFactor
%              deltaDownScaleFactor
            if((currNumOfWhitePixelsInMosqBbs-avgNumOfWhitePixelsInMosqBbs)>diffInPixelsThresh)
                downScaleFactor = downScaleFactor + deltaDownScaleFactor;
            elseif((avgNumOfWhitePixelsInMosqBbs-currNumOfWhitePixelsInMosqBbs)>diffInPixelsThresh)
                downScaleFactor = downScaleFactor - deltaDownScaleFactor;
            else
                if(diffInPixels<=500&&diffInPixels>100)
                    tryCount = tryNtimes;
                    diffInPixelsThresh = 100;
                    deltaDownScaleFactor = deltaDownScaleFactorConst;
                    deltaDownScaleFactor = deltaDownScaleFactor/const1;
                elseif(diffInPixels<=100&&diffInPixels>50)
                    tryCount = tryNtimes;
                    diffInPixelsThresh = 50;
                    deltaDownScaleFactor = deltaDownScaleFactorConst;
                    deltaDownScaleFactor = deltaDownScaleFactor/(const1*const2);
                elseif(diffInPixels<=50&&diffInPixels>10)
                    tryCount = tryNtimes;
                    diffInPixelsThresh = 10;
                    deltaDownScaleFactor = deltaDownScaleFactorConst;
                    deltaDownScaleFactor = deltaDownScaleFactor/(const1*const2*const3);
                else
                    currNumOfWhitePixelsInMosqBbs;
                    avgNumOfWhitePixelsInMosqBbs;
                    break;
                end
            end
            
        end
        %}
        
        
        %when new subtrack is started
        if(startOfCurrentSubTrack&&frameNum>1)
%                      %for first bbs in subtrack, rescale only width and height of bbs to new downScaleFactor which was computed above.
%                      allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(1,3:4) = int16(allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(1,3:4)/downScaleFactor);
%                      
%                      %for remaining bbs in subtrack, rescale x,y, and width and height of bbs to new downScaleFactor which was computed above.
%                      allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(2:end,1) = int16((allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(varJ,1)-allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(varJ-1,1))/downScaleFactor+allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(varJ-1,1));
%                      allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(2:end,2) = int16((allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(varJ,2)-allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(varJ-1,2))/downScaleFactor+allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(varJ-1,2));
%                      allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(2:end,3:4) = int16(allSavedTracksOutput(subTrackCounter).all_bboxes_shifted(varJ,3:4)/downScaleFactor);

        allSavedTracksOutput=rescaleBbs(allSavedTracksOutput,subTrackCounter,downScaleFactor,trackCounter);
        
            
        end
        
        if(enableSizeMatchingBetweenTracks==false)
        %uncomment to ignore all efforts to match sizes between tracks
            downScaleFactor=origDownScaleFactor;
        end
        
        
        
        mosquito_img_resized = imresize(mosquito_img,1/downScaleFactor);
        mosquito_mask_img_resized = imresize(mosquito_mask_img,1/downScaleFactor);
        subTrackFrameNumCounter;
        subTrackCounter;
        allSavedTracksOutput(1).all_bboxes_shifted_output;
        
        if(startOfCurrentSubTrack&&frameNum>1)
%          avgNumOfWhitePixelsInMosqBbs
%          nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold))
%          diffInPixels 
        
%          if(diffInPixels>10)
%              subTrackCounter
%              pause
%          end
%          pause
        
        end
        
        %if you want to use only average of 1st track for entire video, to match sizes between tracks
%          if(subTrackCounter==1)
%              numOfWhitePixelsInMosqBbsInPreviousTrack = numOfWhitePixelsInMosqBbsInPreviousTrack + nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
%  %          numOfWhitePixelsInMosqBbs = nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
%          end

        if(startOfCurrentSubTrack)
            numOfWhitePixelsInMosqBbsInPreviousTrack = nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
        else
            numOfWhitePixelsInMosqBbsInPreviousTrack = numOfWhitePixelsInMosqBbsInPreviousTrack + nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
        end
        
        
        numOfWhitePixelsInMosqBbs = nnz(mosquito_mask_img_resized(mosquito_mask_img_resized>bicubic_pixel_level_threshold));
        
        %This portion code may be commented:
        %%{
        %num of white pixels in mosquito mask
        pixelSizesArray(pixelSizesArrayCounter)  = numOfWhitePixelsInMosqBbs;
        trackIdOfPixelSizesArray(pixelSizesArrayCounter) = subTrackCounter;
        pixelSizesArrayCounter = pixelSizesArrayCounter + 1;
        
%          if(pixelSizesArrayCounter>size(pixelSizesArray,2))
%              save('pixelSizesArray_AvgMosqOfEachTrackResize','pixelSizesArray');
%              save('trackIdOfPixelSizesArray','trackIdOfPixelSizesArray');
%              return
%          end
        %%}
        
%          allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(subTrackFrameNumCounter,:)
%          subTrackCounter
%          subTrackFrameNumCounter
%          pause
        
        curr_bbs = allSavedTracksOutput(subTrackCounter).all_bboxes_shifted_output(subTrackFrameNumCounter,:);
        curr_point = [curr_bbs(1),heightOfimage-curr_bbs(2)];%as Y increases as we go down the image, hence heightOfimage - y.
        
        %get angle between 2 consecutive positions of the mosquito
        angle = calculate_angle(curr_point,prev_point);
        
        
        
%          indexPtr = 2*indexPtr-1;
        pointsForAnglesArray(trackCounter,indexPtr) = prev_point(1);
        pointsForAnglesArray(trackCounter,indexPtr+1) = (heightOfimage-prev_point(2));
        
        %if start of a new long track, don't use previous long track's endpoint, as we will get lines connecting the 2 long tracks
        if(mosquitoEntryDelayFlag)
            indexPtr= 1;
            pointsForAnglesArray(trackCounter,indexPtr) = curr_point(1);
            pointsForAnglesArray(trackCounter,indexPtr+1) = (heightOfimage-curr_point(2));
        end
        pointsForAnglesArray(trackCounter,indexPtr+2) = curr_point(1);
        pointsForAnglesArray(trackCounter,indexPtr+3) = (heightOfimage-curr_point(2));
        indexPtr;
        pointsForAnglesArray(1:2,1:20);
        
        
        
        %num of white pixels in mosquito mask
        anglesArray(anglesArrayCounter)  = angle;
        trackIdOfAnglesArray(anglesArrayCounter) = subTrackCounter;
        anglesArrayCounter = anglesArrayCounter + 1;
        
%          if(anglesArrayCounter>size(anglesArray,2))
%              save('anglesArray_1','anglesArray');
%              save('trackIdOfAnglesArray','trackIdOfAnglesArray');
%              return
%          end
        
        
%          trackCounter
%          subTrackCounter
%          trackIdOfAnglesArray(anglesArrayCounter-1)
%          curr_point
%          prev_point
%          disp('angle')
%          disp(angle)
        
        %when subtrack has increased, and moved to next subtrack
        if(startOfCurrentSubTrack)
            %add a circle at end of previous track
            
            pointsForEndOfSubTracksArray(indexPtr2,1) = prev_point(1);
            pointsForEndOfSubTracksArray(indexPtr2,2) = (heightOfimage-prev_point(2));
            pointsForEndOfSubTracksArray(indexPtr2,3) = 5;
        
            
            
            if(strcmp(line_color,'blue'))
                line_color = 'red';
            else
                line_color = 'blue';
            end
            
            indexPtr2 = indexPtr2 + 1; 
        end
        
        
        %erase circles of previous long track and start afresh
        if(mosquitoEntryDelayFlag)
                indexPtr2 = 1;
                pointsForEndOfSubTracksArray(indexPtr2,1) = curr_point(1);
                pointsForEndOfSubTracksArray(indexPtr2,2) = (heightOfimage-curr_point(2));
                pointsForEndOfSubTracksArray(indexPtr2,3) = 5;
                indexPtr2 = indexPtr2 + 1; 
        end
%          pointsForEndOfSubTracksArray(1:50,:)
%          %when one long sequence of joined track ends and a new one starts elsewhere, add delay to entrance of new track
%          if(matchMosquitoSizeFlag)
%              
%          end
        
        if(drawTrackPathFlag)
            %draw circular point at start of each new subtrack
            image = insertShape(image,'FilledCircle',pointsForEndOfSubTracksArray(1:indexPtr2-1,:),'LineWidth',2,'Color',['green']);
            
            %draw lines between sub tracks
    %          image = insertShape(image,'Line',[0,0,prev_point(1) , (heightOfimage-prev_point(2)), curr_point(1), (heightOfimage-curr_point(2))],'LineWidth',2,'Color',line_color);
            image = insertShape(image,'Line',pointsForAnglesArray(trackCounter,1:indexPtr+3),'LineWidth',2,'Color',['blue']);
        end

        indexPtr = indexPtr + 2;
        
%          pause;

        prev_point = curr_point;
        
        curr_bbs(3) = size(mosquito_img_resized,2);
        curr_bbs(4) = size(mosquito_img_resized,1);
        
%         curr_bbs
        
%          black_img_resized_NN = imresize(black_img,1/downScaleFactor,'nearest');
%          black_img_resized_bicubic = imresize(black_img,1/downScaleFactor);
%          frame2Mask
%          pause


%Use for adding occlusion
        %{
        frame2Mask_cropped = frame2Mask(curr_bbs(2):curr_bbs(2)+curr_bbs(4)-1,curr_bbs(1):curr_bbs(1)+curr_bbs(3)-1,:);
%          frame2Mask_cropped(:,:,:)
        randVal = 0;
        
        
        %if there is a person in the region where the mosquito will be composited
        if( sum(frame2Mask_cropped(:)) > 0 )
            personPresentInCurrFrameFlag = 1;
        else
            personPresentInCurrFrameFlag = 0;
            noOcclFlag = 1;
        end
        %}

        %when mosquito enters a region having a pedestrian
        if( (personPresentInCurrFrameFlag==1)&&(personPresentInPrevFrameFlag==0) )
%              randVal = rand;
%              %if there is a person in the region where the mosquito will be composited, and you want the mosquito to be occluded
%              if(randVal<=0)
%                  noOcclFlag = 0;
%              %if there is a person in the region where the mosquito will be composited, and you want the mosquito to NOT be occluded
%              else 
%                  noOcclFlag = 1;
%              end
            
            %if occlCountDownTimer still active, then use value of prevNoOcclFlag for noOcclFlag
            if(occlCountDownTimer>0)
                noOcclFlag = prevNoOcclFlag;
            else
                if(prevNoOcclFlag==0)
                    noOcclFlag = 1;
                else 
                    noOcclFlag = 0;
                end
            end
            
            
            if(occlCountDownTimer<=0)
                prevNoOcclFlag = noOcclFlag;
                a = 100;
                b = 150;
                %random integer between 250 n 350
                occlCountDownTimer = round(a + (b-a).*rand);
            end

        end
        
        noOcclFlag;
        
        occlCountDownTimer = occlCountDownTimer-1;
        
%          if(frameNum>=150)
%              sum(frame2Mask_cropped(:))
%              noOcclFlag
%              personPresentInCurrFrameFlag
%              personPresentInPrevFrameFlag
%              frameNum
%              pause
%          end
        
        if(noOcclFlag||~personPresentInCurrFrameFlag)
            image_cropped = image(curr_bbs(2):curr_bbs(2)+curr_bbs(4)-1,curr_bbs(1):curr_bbs(1)+curr_bbs(3)-1,:);
    %              image_cropped(black_img_resized_NN ~= 0) = frame_resized(black_img_resized_NN ~= 0);
    %              black_img_resized_bicubic(black_img_resized_NN>0)

    %              image_cropped(black_img_resized_bicubic >= 16) = frame_resized(black_img_resized_bicubic >= 16);
    %              frame_resized(black_img_resized_bicubic >= 16)

            mosquito_img_resized_double = im2double(mosquito_img_resized);
            image_cropped_double = im2double(image_cropped);
            
            %Option 1 and 2 for compositing
            composition_option = 1;
            
            bicubic_pixel_level_threshold = 16;
            
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
            
            image_orig = image;

            if(showMergedTrackNums)
                curr_track_id = allSavedTracksOutput(subTrackCounter).trackId(subTrackFrameNumCounter);
                image_orig = image;
                image = insertObjectAnnotation(image, 'rectangle', ...
                                    curr_bbs, int2str(curr_track_id) );
                writeVideo(obj.writer1,image);

            else
                image = image_orig;
    %              s = int2str(subTrackCounter);
%                  str = strcat(int2str(trackCounter),'_',int2str(subTrackCounter),'_',int2str(numOfWhitePixelsInMosqBbs)) ;
                str = strcat(int2str(trackCounter),'_',int2str(subTrackCounter)) ;

                image = insertObjectAnnotation(image, 'rectangle', ...
                                    curr_bbs, str );
                writeVideo(obj.writer2,image);

                                    
            end
        end
    
      personPresentInPrevFrameFlag = personPresentInCurrFrameFlag;
%          Display the mask and the frame.      
      displayImage(image,interFrameDisplayDelay,stepFlag,displayImageFlag);
      
%          obj.videoPlayer.step(image);
%          obj.maskPlayer.step(image_withBbs);
%          obj.videoPlayer.step(black_img_resized_bicubic);
        
%          save video of mask (grayscale) and frame to file
        writeVideo(obj.writer1,image);
%          writeVideo(obj.writer2,image_withBbs);

    end
   
end