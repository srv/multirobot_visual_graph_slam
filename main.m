
clear all;
close all;
theFigure=figure('units','normalized','outerposition',[0 0 1 1]); % graficar
% SOURCES FOR GITHUB


%% dataset gopro2 y gopro3: descomentar "%%%%%"
% Load externally detected loop closings
loopClosings=load('data/loopclosings.csv'); %   this file contains couple of intra- and inter-session loop closings with the images id. 

% Load odometric data of first session
%load data/odoData1_global;  % caragar datos de archivo odometría para la primer sesión de SLAM
odoData1=load('data/odoData');
odoData2=load('data/odoData2');
numframeT1=226; % number of frames of trajectory 1
numframeT2=199; % number of frames of trajectory 2
N=5; % number of frames that have to be read to perform a local SLAM run. Local SLAM step in number of frames.
minimum_ransac_inliers=53; % minimum number of inliers to consider a transform between images obtained with RANSAC as valid (true positive)  

%%%------------------------

% star and ending image ids.
firstFrame1=odoData1.odoData(1).imNum;
lastFrame1=odoData1.odoData(end).imNum;
lastFrame2=odoData2.odoData(end).imNum;
firstFrame2=odoData2.odoData(1).imNum;




%% define parameters 
number_of_inliers_for_LC=80; % 13, 99 minimum number of inliers to consider an inter-session loop closing proposed by HALOC as valid. 
loopsToJoin=1; %  minimum number of inter-session loops to joint maps 
NofInterSessionLoopClosingsToOptimize=2; % number of inter-session loop closings accumulated to optimitze the global graph 

f1T1=1; 
f2T1=f1T1+N;

f1T2=1; 
f2T2=f1T2+N;
maps_joined=false;
continuing=true; 
slamData1total=[]; 
slamData1total.X=[];              
    % State covariance
slamData1total.P=[];              
% Image number corresponding to frame i.
slamData1total.F=[];              
% SIFT features. theFeatures(i).f and theFeatures(i).d are the
% coordinates and descriptors of the SIFT features of the image in
% frame i.
slamData1total.theFeatures=[];
% Previous SLAM session
slamData1total.slamPrev=[];
% Indexes to the links between maps. Initialized to one to ease plot
% when no links available.
slamData1total.theLinks=1;
% If 1, this session has still not been joined with the previous one.
slamData1total.separatedMaps=1;
% External loops, just for log and plot
slamData1total.externalLoops=[];
% Internal loops, just for log and plot
slamData1total.internalLoops=[];


slamData2total=[]; 
slamData2total.X=[];              
    % State covariance
slamData2total.P=[];              
% Image number corresponding to frame i.
slamData2total.F=[];              
% SIFT features. theFeatures(i).f and theFeatures(i).d are the
% coordinates and descriptors of the SIFT features of the image in
% frame i.
slamData2total.theFeatures=[];
% Previous SLAM session
slamData2total.slamPrev=[];
% Indexes to the links between maps. Initialized to one to ease plot
% when no links available.
slamData2total.theLinks=1;
% If 1, this session has still not been joined with the previous one.
slamData2total.separatedMaps=1;
% External loops, just for log and plot
slamData2total.externalLoops=[];
% Internal loops, just for log and plot
slamData2total.internalLoops=[];
inicio=true; 



externalMeasurementsTotal1.F=[];    
externalMeasurementsTotal1.Fe=[];     
externalMeasurementsTotal1.Z=[]; 

externalMeasurementsTotal.F=[];     
externalMeasurementsTotal.Fe=[];    
externalMeasurementsTotal.Z=[]; 

% create video writer
writerObj = VideoWriter('video_multirobot.avi');
writerObj.FrameRate = 5;
 % open the video writer
open(writerObj); 
%%%% video writer

while maps_joined==false && continuing
	imgPath='./data/images/ValldemossaGopro/'; % images gopro 1,2	
    h1=subplot(1,2,2);
	[lastnode1, slamData1,globalData,global_position,position,lc,externalMeasurementsOUT1,writerObj]=mslam_video_multirobot(theFigure,writerObj, imgPath, h1,loopsToJoin, number_of_inliers_for_LC, externalMeasurementsTotal1, inicio, slamData1total,1, odoData1.odoData,loopClosings,[],f1T1,f2T1); % function local SLAM 
	
	externalMeasurementsTotal1= externalMeasurementsOUT1;
	
	slamData1total=slamData1;
	last_odo1=slamData1total.F(end); % last odometric displacement before map joining
	
	h2=subplot(1,2,1);

    [lastnode2, slamData2,globalData,global_position,position,lc, externalMeasurementsOUT,writerObj]=mslam_video_multirobot(theFigure, writerObj, imgPath, h2,loopsToJoin,number_of_inliers_for_LC, externalMeasurementsTotal, inicio, slamData2total, 2, odoData2.odoData,loopClosings,slamData1total,f1T2,f2T2);

  % accumulate slam data
    externalMeasurementsTotal= externalMeasurementsOUT;
  
	slamData2total=slamData2; 
	last_odo2=slamData2total.F(end)-firstFrame2+1;% index of the last odometric displacement before map joining. Take into account that the number of the 
	% frame in the second dataset starts at frame number 227, in this dataset

    
    
    if inicio
    %	slamData2total.theFeatures=slamData2.theFeatures;
    	inicio=false;
    end;
 		

	if slamData2total.separatedMaps == 1 
		maps_joined=false;
	else
		maps_joined=true;
	end;

	if f2T1 < numframeT1-N %% not yet arrived at the end of trajectory 1. No finish the process%
		f1T1=f2T1+1; % run the local Slam procedure until trajectory 2 finishes, from N to N frames. 
		f2T1=f1T1+N;
	end;
	if f2T2 < numframeT2-N
		f1T2=f2T2+1; 
		f2T2=f1T2+N;
	else %% end of trajectory 2 . End the process.
		continuing=false;
	end; 
end;

[position,pairs_loop_closings_image_nodes,externalMeasurements]=mslam_graph_optim_with_loop_closings_153254(externalMeasurementsTotal, writerObj,imgPath,NofInterSessionLoopClosingsToOptimize,minimum_ransac_inliers,numframeT1,last_odo2, last_odo1, number_of_inliers_for_LC, position, odoData1.odoData, odoData2.odoData, loopClosings, lastnode1, lastnode2, numframeT1,numframeT2)
%[position]=mslam_graph_optim(last_odo2, last_odo1, number_of_inliers_for_LC, position, odoData1.odoData, odoData2.odoData, loopClosings, lastnode1, lastnode2, numframeT1,numframeT2)

close(writerObj)



position = optimizePoseGraph(position,'MaxIterations',150,'MaxTime', 1000, 'FunctionTolerance',1e-16,'VerboseOutput','on');
optimized=true;
fprintf('Optimizing Final Graph \n')
figure;
show(position,'IDs','off');
drawnow;
title('Optimized Final Graph');







