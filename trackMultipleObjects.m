function MotionBasedMultiObjectTrackingExample()



%from video: A06I8031.mp4 onwards
glass_mask_coordinates_all_video = [
   
   
40,900,220,1720;%  MVI_7639.MOV
140,1040,420,1580;%  MVI_7641.MOV
140,1040,420,1580;%  MVI_7642.MOV
140,1040,420,1580;%  MVI_7643.MOV
140,1040,420,1580;%  MVI_7644.MOV

140,1040,420,1580;%  MVI_7645.MOV
140,1040,420,1580;%  MVI_7646.MOV
140,1040,420,1580;%  MVI_7647.MOV
140,1040,420,1560;%  MVI_7648.MOV
160,1040,420,1560;%  MVI_7650.MOV


                            
                            ]; %row_x1,row_x2,col_x1,col_x2 , 1,1020,80,1800;

%video files to be processed
%  input_video_files = [string('A06I8031'),string('A06I8032'),string('A06I9821'),string('A06I9822'),string('A06I9823'),string('A06I9824')];
input_video_files = [string('MVI_7639'),string('MVI_7641'),string('MVI_7642'),string('MVI_7643'),string('MVI_7644'),string('MVI_7645'),string('MVI_7646'),...
                    string('MVI_7647'),string('MVI_7648'),string('MVI_7650')];
                    
%  remaining files


%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04272018/MVI_7651.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04272018/MVI_7652.MOV
%  
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7653.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7654.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7655.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7656.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7657.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7658.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7659.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7660.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7661.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7662.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7663.MOV
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/04282018/MVI_7664.MOV
%  
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/05012018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06062018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06072018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06082018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06112018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06122018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06132018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06222018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06232018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06242018
%  /home/nitin/gatech_courses/special_problem/code_nitk/Mosquito_Dataset/06252018
                          

%  
%  1,1020,80,1800;%MVI_7512
%  1,1020,80,1800;%MVI_7513
%  1,1020,80,1800;%MVI_7514
%  1,1020,80,1800;%MVI_7515
%  120,1000,200,1750;%MVI_7516
%  
%  120,1000,200,1750;%MVI_7517
%  120,1000,200,1750;%MVI_7518
%  120,1000,200,1750;%MVI_7519
%  120,1000,200,1750;%MVI_7520
%  120,990,200,1740;%MVI_7521
%  
%  120,990,200,1740;%MVI_7522
%  120,990,200,1740;%MVI_7523
%  120,990,180,1720;%MVI_7524
%  120,990,180,1720;%MVI_7525
%  120,980,180,1710;%MVI_7526
%  
%  1,1000,40,1840;%MVI_7527
%  1,1000,40,1840;%MVI_7528
%  1,1000,40,1840;%MVI_7529
%  1,1000,40,1840;%MVI_7530
%  1,1000,40,1840;%MVI_7531
%  
%  1,1000,40,1840;%MVI_7532
%  1,1000,60,1840;%MVI_7533
%  40,1050,20,1840;%MVI_7534
%  40,1050,20,1840;%MVI_7535
%  40,1050,20,1840;%MVI_7536
%  
%  40,1050,20,1840;%MVI_7537
%  40,1050,20,1840;%MVI_7538
%  40,1050,20,1840;%MVI_7539
%  10,1030,40,1860;%MVI_7541
%  10,1030,40,1860;%MVI_7542
%  
%  
%  10,1020,20,1840;%MVI_7547
%  10,1020,40,1860;%MVI_7548
%  10,1020,40,1860;%MVI_7549
%  10,1020,20,1800;%MVI_7550
%  10,1020,20,1840;%MVI_7551
%  
%  10,1020,20,1840;%MVI_7552
%  10,1040,40,1880;%MVI_7553
%  10,1040,40,1860;%MVI_7554
%  10,1060,20,1900;%MVI_7556
%  10,1060,20,1900;%MVI_7557
%  
%  10,1060,20,1860;%MVI_7558
%  10,1060,20,1900;%MVI_7559
%  10,1060,20,1900;%MVI_7560

%  60,900,210,1700;%MVI_7561
%  160,960,220,1660;%MVI_7562

%  40,930,220,1740;%  MVI_7563.MOV
%  40,930,220,1740;%  MVI_7564.MOV
%  40,930,200,1720;%  MVI_7565.MOV
%  40,930,200,1720;%  MVI_7566.MOV
%  40,930,200,1720;%  MVI_7567.MOV
%  40,930,140,1680;%  MVI_7568.MOV
%  40,930,140,1680;%  MVI_7569.MOV
%      
%  70,1040,130,1780;%  MVI_7572.MOV
%  70,1040,130,1780;%  MVI_7573.MOV
%  70,1040,130,1780;%  MVI_7574.MOV
%  70,1040,130,1780;%  MVI_7575.MOV
%  70,1040,130,1780;%  MVI_7576.MOV
%  70,1040,130,1780;%  MVI_7577.MOV
%  70,1040,130,1780;%  MVI_7578.MOV
%  70,1040,130,1780;%  MVI_7579.MOV
%  
%  70,1040,130,1780;%  MVI_7580.MOV
%  70,1040,130,1780;%  MVI_7581.MOV
%  70,1040,130,1780;%  MVI_7582.MOV
%  70,880,260,1640;%  MVI_7583.MOV
%  70,880,260,1640;%  MVI_7584.MOV
%  
%  70,880,260,1640;%  MVI_7585.MOV
%  70,880,260,1640;%  MVI_7586.MOV
%  100,1000,180,1700;%  MVI_7587.MOV
%  100,1000,180,1700;%  MVI_7588.MOV
%  100,1000,180,1700;%  MVI_7589.MOV
%  
%  100,1000,180,1700;%  MVI_7590.MOV
%  100,1000,180,1700;%  MVI_7591.MOV
%  100,1000,180,1700;%  MVI_7592.MOV
%  100,1000,180,1700;%  MVI_7593.MOV
%  100,1000,180,1720;%  MVI_7594.MOV

%  100,1040,400,1620;%  MVI_7665.MOV
%  100,1040,400,1620;%  MVI_7666.MOV
%  100,1040,400,1620;%  MVI_7667.MOV
%  100,1040,400,1620;%  MVI_7668.MOV
%  100,1040,400,1620;%  MVI_7669.MOV
%  100,1040,420,1620;%  MVI_7670.MOV
%  100,1040,420,1620;%  MVI_7671.MOV
%  100,1040,360,1580;%  MVI_7672.MOV
%  100,1040,360,1580;%  MVI_7673.MOV



    



for varI = 1:size(input_video_files,2)

    tracks = initializeTracks(); % Create an empty array of tracks.
    allSavedTracks = initializeTracks(); % Create an empty array of tracks, which will hold all valid tracks during entire video

    nextId = 1; % ID of the next track

    frameNum=0;
    numOfFramesInVideo = 0;%number of video frames in folder
    runNFrames = numOfFramesInVideo;
    runTillEndFlag = true;
    skipNFrame = 0;
    frameRate = 30;%fps
    skipNSeconds = skipNFrame/frameRate;

    glass_mask_coordinates = glass_mask_coordinates_all_video(varI,:); %row_x1,row_x2,col_x1,col_x2

    % Create System objects used for reading video, detecting moving objects,
    % and displaying the results.
    obj = setupSystemObjects();
    
    input_video_files(varI)
    
    %write video to file
    open(obj.writerFrame)
    open(obj.writerBG)


    % Detect moving objects, and track them across video frames.
    %  while ~isDone(obj.reader)
    while hasFrame(obj.reader)
        frame = readFrame(obj.reader);
        
        frameNum = frameNum + 1
        
        if(frameNum==runNFrames&&runTillEndFlag==false)
            break
        end

        %obsolete, as I use skipNSeconds in VideoReader now
    %      if(frameNum<=skipNFrame)
    %          continue
    %      end
        
        
        [centroids, bboxes, mask] = detectObjects(frame);
        centroids_copy = centroids;
        
        predictNewLocationsOfTracks();
        
        [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment();

        updateAssignedTracks();
        updateUnassignedTracks();
        deleteLostTracks();
        createNewTracks();

        %comment following line if you just want to save tracks
        displayTrackingResults(centroids_copy);
    end

    %Save any remaining tracks still active at end of video
    saveValidActiveTracksBeforeClose()

    %save tracksMat to file
    save(obj.tracksFileName,'allSavedTracks');


    %write video to file
    close(obj.writerFrame)
    close(obj.writerBG)

end

    function obj = setupSystemObjects()
        % Initialize Video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        root_folder = '/home/nitin/gatech_courses';
        
        % Create a video file reader.
        
%          obj.reader = VideoReader(strcat('/media/nitin/Elements/Mosquito_Dataset/04122018/',char(input_video_files(varI)),'.MOV'),'CurrentTime',skipNSeconds);
        obj.reader = VideoReader(strcat(root_folder,'/special_problem/code_nitk/lab_videos/new_Setup/mov/',char(input_video_files(varI)),'.MOV'),'CurrentTime',skipNSeconds);
%          obj.reader = vision.VideoFileReader('/nethome/nkodialbail3/cv_special_problem/lab_videos/A06I8030.mp4')
        
        % Create 2 video file writers, one for color frame, another for bg frame.
        obj.writerFrame = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input_video_files(varI)),'_output_multi'));%appends .avi automatically
        obj.writerBG = VideoWriter(strcat(root_folder,'/special_problem/code_nitk/videos/',char(input_video_files(varI)),'_output_multi_BG'));%appends .avi automatically
        
        %
        obj.tracksFileName = strcat(root_folder,'/special_problem/code_nitk/tracks/',char(input_video_files(varI)),'_output_multi.mat');

        % Create two video players, one to display the video,
        % and one to display the foreground mask.
        obj.glassMaskPlayer = vision.VideoPlayer('Position', [0,0,1920,1080]);
        obj.maskPlayer = vision.VideoPlayer('Position', [0,0,1920,1080]);
        obj.videoPlayer = vision.VideoPlayer('Position', [0,0,1920,1080]);

        % Create System objects for foreground detection and blob analysis

        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background.

         %Values for A06I8030.mp4
%          obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
%              'NumTrainingFrames', 150, 'MinimumBackgroundRatio', 0.7, 'LearningRate', 1e-20);
        
        %Values for A06I8031.mp4
        obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
            'NumTrainingFrames', 300, 'MinimumBackgroundRatio', 0.7, 'LearningRate', 1e-20, 'AdaptLearningRate', true);


        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis System object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteristics, such as area, centroid, and the bounding box.

        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 400);
    end
    
    function tracks = initializeTracks()
        % create an empty array of tracks
        tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {}, ...
            'frameNums', {}, ...
            'all_bboxes', {});
    end
    
%      function frame = readFrame()
%          frame = obj.reader.step();
%      end

    %Background subtraction and blob analysis
    function [centroids, bboxes, mask] = detectObjects(frame)

        % Detect foreground.
        mask = obj.detector.step(frame);

        % Apply morphological operations to remove noise and fill in holes.
        mask = imopen(mask, strel('rectangle', [3,3]));
        mask = imopen(mask, strel('rectangle', [3,3]));
%          mask = imopen(mask, strel('rectangle', [3,3]));

        
        mask = imclose(mask, strel('rectangle', [3, 3]));
        mask = imclose(mask, strel('rectangle', [3, 3]));
%          mask = imclose(mask, strel('rectangle', [15, 15]));

        
        mask = imfill(mask, 'holes');
        
        ones_of_mask = mask(mask>0);
        num_of_white_pixels_mask = sum(ones_of_mask(:));
        
        %if lighting changes too much, reset gmm
        if(num_of_white_pixels_mask>15000)
            obj.detector.reset();
            %make mask as 0 when lighting changes too much
            mask(:)=0;
        end
        
%   Do not consider beyond glass walls
%          class(mask)
        glass_mask = mask;
        glass_mask(:) = 0;
        glass_mask(glass_mask_coordinates(1):glass_mask_coordinates(2),glass_mask_coordinates(3):glass_mask_coordinates(4)) = 1;
        mask = mask .* glass_mask;
        mask = logical(mask);
        glass_mask = mask(glass_mask_coordinates(1):glass_mask_coordinates(2),glass_mask_coordinates(3):glass_mask_coordinates(4));
        
        %clear white pixels at the border (defined by glass box) connected by 8-connectivity
        glass_mask = imclearborder(glass_mask,8);
        mask(glass_mask_coordinates(1):glass_mask_coordinates(2),glass_mask_coordinates(3):glass_mask_coordinates(4)) = glass_mask;

%          class(mask)
        
        % Perform blob analysis to find connected components.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
    end

    function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;

            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);

            % Shift the bounding box so that its center is at
            % the predicted location.
            predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
            curr_bbox = [predictedCentroid, bbox(3:4)];
            %enlarge bbs by 1.5x or 2x , so that entire mosquito body is within bbs
%              curr_bbox = resizeBbsByNx(curr_bbox,frame);
            tracks(i).bbox = curr_bbox;
            tracks(i).all_bboxes = cat(1,tracks(i).all_bboxes, curr_bbox);
            tracks(i).frameNums = [tracks(i).frameNums, frameNum];
        end
    end
    
    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()

        nTracks = length(tracks);
        nDetections = size(centroids, 1);

        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end

        % Solve the assignment problem.
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
    end

    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);

            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);

            % Replace predicted bounding box with detected
            % bounding box.
            %enlarge bbs by 1.5x or 2x , so that entire mosquito body is within bbs
            bbox = resizeBbsByNx(bbox,frame);
            tracks(trackIdx).bbox = bbox;
            tracks(trackIdx).all_bboxes(end,:) = bbox; %[tracks(trackIdx).all_bboxes, bbox]
            
%  %  %              disp('debug0')
%  %  %              tracks(trackIdx).all_bboxes
%  %  %              tracks(trackIdx).frameNums
%  %  %              disp('debug1')
            
            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;

            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
    end
    
    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;

            %remove predicted bbs from all_bboxes of tracks(ind), as there was no detection there, but bbox will still show up in video as predicted, but won't be saved. Remove framenum where it was only predicted as well
            tracks(ind).all_bboxes(end,:) = [];
            tracks(ind).frameNums(end) = [];

        end
    end

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end
        
        %this is kept 1, so that any time, only prediction occurs without an accompanying detection, end that track and start a new track when detection reoccurs
        invisibleForTooLong = 1;%4
        
        %allows only tracks having minimum of 5 frames in which mosquitoes were predicted and detected as well. i.e. min length of each track will be >=5
        ageThreshold = 6;

        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;

        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

        % Find the indices of 'lost' tracks, which lasted more than ageThreshold
        lostValidInds = (ages >= ageThreshold) & ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
                
        %Save valid tracks before deletion
        allSavedTracks = [allSavedTracks , tracks(lostValidInds)];
        
        % Delete lost tracks.
        tracks = tracks(~lostInds);
        
    end
    
    
    %After running through loop of frames, save any remaining active valid tracks
    function saveValidActiveTracksBeforeClose()
        if isempty(tracks)
            return;
        end

        ageThreshold = 8;

        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        
        % Find the indices of 'valid' tracks which are still active, which lasted more than ageThreshold
        validInds = (ages >= ageThreshold) ;

        %Save valid tracks before deletion
        allSavedTracks = [allSavedTracks , tracks(validInds)]
        
    end
    
    function resizedBbs = resizeBbsByNx(bbs,frame)
%          img_height = size(frame,1);
%          img_width = size(frame,2);
        rescaleRatio = 2.0;
        tmp = bbs;
        tmp(3) = rescaleRatio*bbs(3);
        tmp(4) = rescaleRatio*bbs(4);
        tmp(1) = tmp(1) - (rescaleRatio-1)/2*bbs(3);
        tmp(2) = tmp(2) - (rescaleRatio-1)/2*bbs(4);
%          if(tmp(1)<1)
%              tmp(1) = 1;
%          end
%          if(tmp(2)<1)
%              tmp(2) = 1;
%          end
%          if(tmp(1)+tmp(3)>=img_height)
%              tmp(3) = img_height - tmp(1) - 1;
%          end
%          if(tmp(2)+tmp(4)>=img_width)
%              tmp(4) = img_width - tmp(2) - 1;
%          end
%          bbs
        resizedBbs = tmp;
    end
    function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);

        for i = 1:size(centroids, 1)

            centroid = centroids(i,:);
            bbox = bboxes(i, :);

            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantAcceleration', ...
                centroid, 1E5 * ones(1, 3), [600, 100, 10], 25);
                
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0, ...
                'frameNums', [frameNum], ...
                'all_bboxes', [bbox] );

            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
%              fprintf('frame num %i id %i\n',frameNum,nextId);
            % Increment the next id.
            nextId = nextId + 1;
        end
    end
    
    function displayTrackingResults(centroids_copy)
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask_RGB = uint8(repmat(mask, [1, 1, 3])) .* 255;
        mask = uint8(mask).*255;
        
        minVisibleCount = 1; %8
        if ~isempty(tracks)

            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than
            % a minimum number of frames.
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);

            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox);
%                  pause;

                % Get ids.
                ids = int32([reliableTracks(:).id]);

                % Create labels for objects indicating the ones for
                % which we display the predicted rather than the actual
                % location.
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);

                % Draw the objects on the frame.
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);
%                      bboxes
                
                %display center of tracks calculated as green + marks
                frame = insertMarker(frame,bboxes(:,1:2)+bboxes(:,3:4)/2,'+','color','green','size',10);
                
              
                
                % Draw the objects on the mask.
                mask_RGB = insertObjectAnnotation(mask_RGB, 'rectangle', ...
                    bboxes, labels);
                
                %display center of tracks calculated as green + marks
                mask_RGB = insertMarker(mask_RGB,bboxes(:,1:2)+bboxes(:,3:4)/2,'+','color','green','size',10);
               
            end
        end
        
%          centroids_copy
        
        %display center of BG detections as red circles
        frame = insertMarker(frame,centroids_copy(:,1:2),'o','color','red','size',10);
                
        %display center of BG detections as red circles
        mask_RGB = insertMarker(mask_RGB,centroids_copy(:,1:2),'o','color','red','size',10);
                
%       Display the area which is being processed, i.e. only inside the glass box covered by white background                    
%          glass_mask = frame;
%          glass_mask(glass_mask_coordinates(1):glass_mask_coordinates(2),glass_mask_coordinates(3):glass_mask_coordinates(4)) = 255;
%          obj.glassMaskPlayer(glass_mask)

%       Display the mask and the frame.
%          step(obj.maskPlayer,mask);
%          obj.maskPlayer.step(mask);
%          obj.videoPlayer.step(frame);
        
        %save video of mask (grayscale) and frame to file
        writeVideo(obj.writerFrame,frame);
        writeVideo(obj.writerBG,mask);
    end
end
