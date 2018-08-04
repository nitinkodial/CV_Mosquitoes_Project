%take all tracks created by trackMultipleObjects obtained from all the lab videos and create new joint/concatenated tracks. Where in each track
%consists of multiple sub tracks concatenated by matching size and angle. Each track has an appropriate entry point in a video frame and exits the video frame in the end.
%The output of this function depends of the resolution of video frame onto which mosquitoes need to be composited.

%function to display downsized mosquito tracks on videos frames of a given dataset. This code uses joining of multiple tracks, to increase the area of tracks
%This composites 0 to 'n' mosquitoes onto the video per frame. This code implements a random number of tracks being composited onto the video at any given period of time. It 
%can be from 0 to n (n is usually 6)




function joinTracks(imgWidth,imgHeight,downScaleFactor)

    lab_videos_tracks = [string('MVI_7500'),string('MVI_7501'),string('MVI_7502'),string('MVI_7503'),string('MVI_7505'),string('MVI_7506'),string('MVI_7507'),string('MVI_7509'),string('MVI_7511')];
    root_folder = '/home/nitin/gatech_courses/';
    lab_videos_tracks_path = 'special_problem/code_nitk/tracks/working_tracks/';
    lab_videos_tracks_bbs_path = 'special_problem/code_nitk/mosquito_bbs/';
    
    allTracks = [];
    allJoinedTracks = [];
    
    for varI = 1:size(lab_videos_tracks,2) 
        inputTracksFileName = strcat(root_folder,lab_videos_tracks_path,char(lab_videos_tracks(varI)),'_output_multi.mat');
        load(inputTracksFileName);
        currTrack = allSavedTracks;

        %add video name and path to rgb and mask bbs of track
        currTrack = addPathToTrack(currTrack,lab_videos_tracks(varI),root_folder,lab_videos_tracks_bbs_path);
        
        %concatenate tracks of different videos into 1
        allTracks = combineTwoTracks(allTracks,currTrack);
    end

    %create new field called all_bboxes_shifted to hold zero origin shifted bbs:
    [allTracks(:).all_bboxes_shifted]=deal(zeros(1,4));

    
    %create new field called all_bboxes_downscaled to downscale the zero origin shifted tracks:
    [allTracks(:).all_bboxes_downscaled]=deal(zeros(1,4));
    
%      save('allTracks','allTracks');
    
    imgWidth = 704;
    imgHeight = 480;
    downScaleFactor = 25;

%      while(size(allTracks,2)>0)
    for varI = 1:size(allTracks,2)
        subTrackIndex = varI;
        [allTracks,edgeIndex] = findBestEdgeOfFrameToEnter(allTracks,subTrackIndex,imgWidth,imgHeight,downScaleFactor);
        allJoinedTracksCurr = cleanAllJoinedTracks(allTracks(subTrackIndex));
        allJoinedTracks = [allJoinedTracks,allJoinedTracksCurr];
    end
    save('allTracks','allTracks');

    
    allTracks = findAnglesBetweenTracks(allJoinedTracks,imgWidth,imgHeight)
    allTracks = matchAnglesBetweenTracks(allTracks);
    
%      disp('d0')
    %display some of the exit and following entry angles
%      for varI = 1:20
%          [allTracks(varI).exitAngle,allTracks(varI+1).entryAngle,size(allTracks(varI).all_bboxes_downscaled,1)]
%      end
    save('allTracks','allTracks');
    
end

%rearrange subtracks in allTracks such that exit angle of current track matches entry angle of next track.
function [allTracks] = matchAnglesBetweenTracks(allTracks)
    allTracksRearranged = [];
    
%      indices = [1:size(allTracks,2)];
%      [allTracks(:).entryAngle]
%      [allTracks(:).exitAngle]
    entryAngles = [allTracks(:).entryAngle];
    for varI = 1:size(allTracks,2)-1
        entryAngles(1) = [];
%          indices(varI) = [];
        
        currExitAngle = [allTracks(varI).exitAngle];
        
        [minDiff minDiffIndex] = min(abs(entryAngles-currExitAngle));
        
        %swap minDiffIndex with 1st index
        tmp = entryAngles(minDiffIndex);
        entryAngles(minDiffIndex) = entryAngles(1);
        entryAngles(1) = tmp;
        
        tmp = allTracks(minDiffIndex+varI);
        allTracks(minDiffIndex+varI) = allTracks(varI+1);
        allTracks(varI+1) = tmp;
        
%          disp('d0')
%          currExitAngle
%          entryAngles(minDiffIndex)
%          allTracks(varI+minDiffIndex).entryAngle

%          if(minDiffIndex<varI)
%              disp('d1')
%              allTracks(minDiffIndex).entryAngle
%          else
%              disp('d2')
%              allTracks(minDiffIndex+1).entryAngle
%          end
%          entryAngles = [allTracks(:).entryAngle];
%          pause
    end

end

%find entry and exit angle of every subtrack
function [allTracks] = findAnglesBetweenTracks(allTracks,imgWidth,imgHeight)
    
    p2 = [0,0];
    p1 = [0,0];
    
    for varI = 1:size(allTracks,2)
%          disp(allTracks(varI).all_bboxes_downscaled)
        %calculate sub track's entry angle
        p2 = allTracks(varI).all_bboxes(2,1:2);
        p1 = allTracks(varI).all_bboxes(1,1:2);
        p2(2) = imgHeight - p2(2);
        p1(2) = imgHeight - p1(2);
        
        angle = calculate_angle(p2,p1);
        if(angle<0)
            %if angle is between -180 and 0 , make it positive by adding 360 to it
            angle = 360+angle;
        end

        allTracks(varI).entryAngle = angle;
        
        %calculate sub track's exit angle
%          lastIndex = size( allTracks(varI).all_bboxes_downscaled,1 );
        p2 = allTracks(varI).all_bboxes(end,1:2);
        p1 = allTracks(varI).all_bboxes(end-1,1:2);
        p2(2) = imgHeight - p2(2);
        p1(2) = imgHeight - p1(2);
        
        angle = calculate_angle(p2,p1);
        
        if(angle<0)
            %if angle is between -180 and 0 , make it positive by adding 360 to it
            angle = 360+angle;
        end

        allTracks(varI).exitAngle = angle;
    
    end
end





%remove old unnecessary fields from allJoinedTracksCurr and add downScaleFactor and path for rgb and mask bbs for every mosquito bbs individually. 
function [allJoinedTracksCurr] = cleanAllJoinedTracks(allJoinedTracksCurr)

    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'bbox');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'kalmanFilter');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'age');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'totalVisibleCount');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'consecutiveInvisibleCount');
%      allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'all_bboxes');% this is required to calculate exit and entry angles accurately
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'all_bboxes_shifted');
    
    allJoinedTracksCurr.mask_bbs_paths = [] ;
    allJoinedTracksCurr.rgb_bbs_paths = [];
    allJoinedTracksCurr.downScaleFactors = [] ;

    allJoinedTracksCurr(:).all_bboxes = double(allJoinedTracksCurr(:).all_bboxes);
    lengthOfSubTrack = size(allJoinedTracksCurr.all_bboxes_downscaled);
    for varI = 1:lengthOfSubTrack
        allJoinedTracksCurr.mask_bbs_paths = [ allJoinedTracksCurr.mask_bbs_paths , strcat( allJoinedTracksCurr.mask_bbs_path , num2str(allJoinedTracksCurr.frameNums(varI)) , '_' , num2str(allJoinedTracksCurr.id) )];
        allJoinedTracksCurr.rgb_bbs_paths = [ allJoinedTracksCurr.rgb_bbs_paths , strcat(allJoinedTracksCurr.rgb_bbs_path,num2str(allJoinedTracksCurr.frameNums(varI)),'_',num2str(allJoinedTracksCurr.id))];
        allJoinedTracksCurr.downScaleFactors = [allJoinedTracksCurr.downScaleFactors,allJoinedTracksCurr.downScaleFactor];
    end
    
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'id');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'mask_bbs_path');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'rgb_bbs_path');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'downScaleFactor');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'videoName');
    allJoinedTracksCurr = rmfield(allJoinedTracksCurr,'frameNums');
    
end

%shift bboxes of mosquitoes at the time of recording, to origin.
function [allTracks] = zeroShiftTracks(allTracks,subTrackIndex)
    
    %allTracks(subTrackIndex).all_bboxes_shifted = zeros(allTracks(subTrackIndex).totalVisibleCount,4);
    for varJ = 1:(allTracks(subTrackIndex).totalVisibleCount)
        if(varJ)==1
            allTracks(subTrackIndex).all_bboxes_shifted(varJ,1:2) = 1;
        else
            allTracks(subTrackIndex).all_bboxes_shifted(varJ,1) = allTracks(subTrackIndex).all_bboxes(varJ,1)-allTracks(subTrackIndex).all_bboxes(varJ-1,1)+allTracks(subTrackIndex).all_bboxes_shifted(varJ-1,1);
            allTracks(subTrackIndex).all_bboxes_shifted(varJ,2) = allTracks(subTrackIndex).all_bboxes(varJ,2)-allTracks(subTrackIndex).all_bboxes(varJ-1,2)+allTracks(subTrackIndex).all_bboxes_shifted(varJ-1,2);
        end
            
            %bbox size remains same inspite of shift in x,y co-ordinates
            allTracks(subTrackIndex).all_bboxes_shifted(varJ,3:4) = allTracks(subTrackIndex).all_bboxes(varJ,3:4);
        
    end

end

%downscale the zero shifted mosquitoes coordinates. This includes downscaling distances between consecutive positions of a track as well as downscaling the bbs size.
function [allTracks] = downscaleTracks(allTracks,subTrackIndex,downScaleFactor,startcol,startrow)
    
    %startrow,%startcol
%      startrow = 10;
%      startcol = 10;
    
    %allTracks(subTrackIndex).all_bboxes_shifted = zeros(allTracks(subTrackIndex).totalVisibleCount,4);
    for varJ = 1:(allTracks(subTrackIndex).totalVisibleCount)
        if(varJ)==1
            allTracks(subTrackIndex).all_bboxes_downscaled(varJ,1:2) = [startcol,startrow];
        else
            allTracks(subTrackIndex).all_bboxes_downscaled(varJ,1) = int16((allTracks(subTrackIndex).all_bboxes_shifted(varJ,1)-allTracks(subTrackIndex).all_bboxes_shifted(varJ-1,1))/downScaleFactor ...
                                                                    + allTracks(subTrackIndex).all_bboxes_downscaled(varJ-1,1));
            
            allTracks(subTrackIndex).all_bboxes_downscaled(varJ,2) = int16((allTracks(subTrackIndex).all_bboxes_shifted(varJ,2)-allTracks(subTrackIndex).all_bboxes_shifted(varJ-1,2))/downScaleFactor ...
                                                                    + allTracks(subTrackIndex).all_bboxes_downscaled(varJ-1,2));
        end
            allTracks(subTrackIndex).all_bboxes_downscaled(varJ,3:4) = int16(allTracks(subTrackIndex).all_bboxes_shifted(varJ,3:4)/downScaleFactor);
        
    end

end


%best edge among top,left,right and bottom of video frame, where mosquito when enters lasts longest within bounds of the frame
function [allTracks,edgeIndex] = findBestEdgeOfFrameToEnter(allTracks,subTrackIndex,imgWidth,imgHeight,downScaleFactor)

    edgeMidPoints = zeros(4,2);
    gap = 10;
    %left edge midpoint with some gap
    edgeMidPoints(1,:) = [gap,int16(imgHeight/2)];
    %top edge midpoint with some gap
    edgeMidPoints(2,:) = [int16(imgWidth/2),gap];
    %right edge midpoint with some gap for bbs
    edgeMidPoints(3,:) = [imgWidth-2*gap,int16(imgHeight/2)];
    %bottom edge midpoint with some gap for bbs
    edgeMidPoints(4,:) = [int16(imgWidth/2),imgHeight-2*gap];
    
    
    allTracks = zeroShiftTracks(allTracks,subTrackIndex);
    allTracks_Orig = allTracks;
    
    longestCounterVal = 0;
    edgeIndex = 0;
    for varI = 1:size(edgeMidPoints,1)
        allTracks = allTracks_Orig;
        allTracks = downscaleTracks(allTracks,subTrackIndex,downScaleFactor,edgeMidPoints(varI,1),edgeMidPoints(varI,2));
        counter = 0;
        for varJ = 1:(allTracks(subTrackIndex).totalVisibleCount)
            
%              allTracks(subTrackIndex).all_bboxes_downscaled(varJ,:)
            
            if(allTracks(subTrackIndex).all_bboxes_downscaled(varJ,1)<1) || (allTracks(subTrackIndex).all_bboxes_downscaled(varJ,2)<1)
%                  disp('d0');
                break;
            end
            if((allTracks(subTrackIndex).all_bboxes_downscaled(varJ,1)+allTracks(subTrackIndex).all_bboxes_downscaled(varJ,3))>imgWidth) 
%                  disp('d1');
                break;
            end
            if ((allTracks(subTrackIndex).all_bboxes_downscaled(varJ,2)+allTracks(subTrackIndex).all_bboxes_downscaled(varJ,4))>imgHeight)
%                  disp('d2');
                break;
            end
            counter = counter + 1;
        end
%          counter
        if(counter>longestCounterVal)
            edgeIndex = varI;
            longestCounterVal = counter;
        end
    end %
%      edgeIndex
%      longestCounterVal
    allTracks = downscaleTracks(allTracks,subTrackIndex,downScaleFactor,edgeMidPoints(edgeIndex,1),edgeMidPoints(edgeIndex,2));
    allTracks(subTrackIndex).downScaleFactor = downScaleFactor;
end %function

%concatenate tracks of different videos into 1
function [combinedTrack] = combineTwoTracks(combinedTrack,newTrack)
    combinedTrack = [combinedTrack,newTrack];
end

%add video name and path to rgb and mask bbs of track
function [track] = addPathToTrack(track,trackName,root_folder,trackBbsPath)

    [track(:).videoName] = deal(trackName);
    [track(:).downScaleFactor] = deal(0);
    [track(:).entryAngle] = deal(0);
    [track(:).exitAngle] = deal(0);
    [track(:).mask_bbs_path] = deal( strcat(root_folder,trackBbsPath,trackName,'/masks/',trackName,'_'));
    [track(:).rgb_bbs_path] = deal( strcat(root_folder,trackBbsPath,trackName,'/rgb/',trackName,'_'));
    
end




