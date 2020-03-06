% Name        : =mslam_graph_optim(odoData,loopClosings,slamPrev,firstFrame,lastFrame)
% Description : 2D Graph-Based Visual SLAM with two joined different sessions in a multirobot context

% Input       : odoData1, and odoData2 - Odometric data of trajectories 1 and 2 as provided by
%                            compute_odometry. See https://github.com/aburguera/VISUAL_ODOMETRY_2D
%                            for more information. 

%               loopClosings loop closings obtained by external means. Format
%                            is: first column and second column are the
%                            image numbers (imNum) of the images that close
%                            the loop. Third column states the quality of
%                            the loop (the higher the better), though is
%                            not used in this implementation.
%		These odometries and the inter-session loop closings are the unique data input to this graph formation node. 
%		From now on, the 'search_loops" will be done only between inter-session loops. 
%       the optimization process will be done only over a single trajectory. 
%               position   - graph to be completed and optimized
%               firstFrame - Index within odoData of the first odometric
%                            estimate to use to perform SLAM.
%               lastFrame  - Index within odoData of the last odometric
%                            estimate to use to perform SLAM.
%       number_of_inliers_for_LC - minimum number of inliers to consider an inter-session loop closing as a valid. 
%       last_odoi - last odometry trajectory "i" 
%       last_framei - last frame sequence "i" 
% Output      : position: graph with nodes and edges
% pairs_loop_closings_image_nodes: set of correspondences between image number and graph nodes: [inNum edgeID]
% externalMeasurements: 2D transform between inter-session loop closings 
% Author      : Francisco Bonin Font francisco.bonin@uib.es
%               antoni.burguera@uib.es
% Note        : Please, refer to the README.TXT file for information about
%               how to properly cite us if you use this software.

% be carreful: features of frame i are in odoData(i).f;


function [position, pairs_loop_closings_image_nodes, externalMeasurements]=mslam_graph_optim_with_loop_closings_153254(externalMeasurementsTotal, writerObj,imgPath, NofInterSessionLoopClosingsToOptimize, minimum_ransac_inliers,numframeT1,last_odo2, last_odo1, number_of_inliers_for_LC, position, odoData1, odoData2, loopClosings, lastnode1, lastnode2, lastFrame1,lastFrame2)
    
    theFigure=figure('units','normalized','outerposition',[0 0 1 1], 'position', [0 0 1 1] )
    %theFigure=figure;
    axes1=gca; % current axes (the ones of h)
    %axes1=axes('pos',[-.1 .0]);
    % axes2=axes('pos',[.04 .6 .5 .3]); % original [.65 .70 .45 .25]
    axes2=axes('pos',[.65 .70 .45 .25]); % imatge superior new axes located in 'pos'
    axes3=axes('pos',[.65 .15 .45 .25]); % imatge inferior
    open(writerObj); 
    lc = [];
    lc=loops_closings(loopClosings,number_of_inliers_for_LC); % keep only the loop closing candidates that have more than more than "number_of_inliers_for_LC" inliers. 
    % Initialize system
    globalData1=init(odoData1,lc, last_odo1, lastFrame1);  % init initial parameters  
    % : first frame= last odometry before map joining. "lastFrame": last frame of the sequence. 
    

    globalData2=init(odoData2,lc,last_odo2, lastFrame2); % store odometry of trajectory 2.  
    optimized=false;
    n=1;
    global_position = [];
    externalMeasurements = [];  
    
    comptaLC=0;
    oldsizeExternalMeasurements=0;
    
    pairs_loop_closings_image_nodes=[]; % vector that contains images and the corresponding nodes of each loop closing. 

    while more_frames(globalData1)
        global_position=globalData1.odoData(globalData1.curFrame).X; % last displacement of the vector state in the form of a transform (x,y,theta)
        image_of_new_node= globalData1.odoData(globalData1.curFrame).imNum; % its corresponding image
        [edge,edgeIDtmp]=addRelativePose(position,global_position,[10 0 0 10 0 10],lastnode1); % add a pose in the graph, from the last node of trajectory 1 with the transform included in global_position
        lastnode1=edge(2); % the second component of "edge" contains the node Id.
        pairs_loop_closings_image_nodes(n,1)= image_of_new_node; % store image and node Id, both related
        pairs_loop_closings_image_nodes(n,2)= edge(2); 
        n=n+1; 
        globalData1=next_frame(globalData1); % process the next frame and add a new node with the next displacement
        if more_frames(globalData1)
         global_position=globalData1.odoData(globalData1.curFrame).X; 
         image_of_new_node= globalData1.odoData(globalData1.curFrame).imNum;
         [edge,edgeID1]= addRelativePose(position,global_position,[10 0 0 10 0 10]); 
         pairs_loop_closings_image_nodes(n,1)= image_of_new_node; 
         pairs_loop_closings_image_nodes(n,2)= edge(2); 
         n=n+1; 
         lastnode1=edge(2); 
         globalData1=next_frame(globalData1);
        end

        
        imFileName=[imgPath num2str(image_of_new_node) '.png'];
        curImage=imread(imFileName);
        % Plot SLAM and images of each new node.
        axes(axes1);
        %cla; % clear axes
        show(position,'IDs','off');
        if optimized
            title('Optimized Multi Robot Graph ');
        else
            title('Non-Optimized Multi Robot Graph');
        end;
        %title('Non-Optimized Multi Robot Graph');
        axes(axes2);
        %cla;
        imshow(curImage);

        drawnow;
        title('Image of Trajectory 1');

        frame=getframe(theFigure);
        writeVideo(writerObj, frame);


        
        
        if more_frames(globalData2)   % if second trajectory is still running
            %search for inter-session loop closings 
            [externalMeasurements,position]=search_loops(minimum_ransac_inliers,position, pairs_loop_closings_image_nodes,numframeT1,globalData1, globalData2,externalMeasurements);         
            comptaLC=size(externalMeasurements,1)-oldsizeExternalMeasurements+comptaLC;
            oldsizeExternalMeasurements=size(externalMeasurements,1);
            global_position=globalData2.odoData(globalData2.curFrame).X;
            image_of_new_node= globalData2.odoData(globalData2.curFrame).imNum;
            [edge,edgeIDtmp]=addRelativePose(position,global_position,[10 0 0 10 0 10],lastnode2); 
            lastnode2=edge(2);
            pairs_loop_closings_image_nodes(n,1)= image_of_new_node; 
            pairs_loop_closings_image_nodes(n,2)= edge(2); 
            n=n+1; 
            globalData2=next_frame(globalData2);% next frame
            if more_frames(globalData2)
                global_position=globalData2.odoData(globalData2.curFrame).X;
                image_of_new_node= globalData2.odoData(globalData2.curFrame).imNum;
                [edge,edgeID2]= addRelativePose(position,global_position,[10 0 0 10 0 10]); % add relative pose from the last node
                pairs_loop_closings_image_nodes(n,1)= image_of_new_node; 
                pairs_loop_closings_image_nodes(n,2)= edge(2); 
                n=n+1; 
                lastnode2=edge(2); 
                %search for external loops
                [externalMeasurements,position]=search_loops(minimum_ransac_inliers,position, pairs_loop_closings_image_nodes,numframeT1,globalData1, globalData2,externalMeasurements);
                comptaLC=size(externalMeasurements,1)-oldsizeExternalMeasurements+comptaLC;
                oldsizeExternalMeasurements=size(externalMeasurements,1);
                globalData2=next_frame(globalData2);
                comptaLC
            end

            imFileName=[imgPath num2str(image_of_new_node) '.png'];
            curImage=imread(imFileName);
            % Plot SLAM
            axes(axes1);
            %cla; % clear axes
            show(position,'IDs','off');
            drawnow;
            if optimized
                title('Optimized Multi Robot Graph');
            else
                title('Non-Optimized Multi Robot Graph');
            end;
            %title('Non-Optimized Multi Robot Graph');
            axes(axes3);
            %cla; 
            imshow(curImage);

            
            title('Image of Trajectory 2');
            frame=getframe(theFigure);
            writeVideo(writerObj, frame);

            
            
            
            % add the external loops as pose constraints in the graph and optimize it.
            if comptaLC>=NofInterSessionLoopClosingsToOptimize
                diff=(size(externalMeasurements,1)-comptaLC)+1; % diffmust be >0. if comptaLC=size(externalMeasurements,1), add 1
                for i=diff:size(externalMeasurements,1)
                    i1=externalMeasurements(i).im1; % loop closing images
                    i2=externalMeasurements(i).im2; 
                    % nodes corresponding to loop closing images. 
                    image_pair_node1= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i1,:) 
                    image_pair_node2= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i2,:) 
                    if ~isempty(image_pair_node1) && ~isempty(image_pair_node2)
                        node1=image_pair_node1(1,2)  
                        node2=image_pair_node2(1,2) 
                        Z=externalMeasurements(i).transform;
                        addRelativePose(position,Z,[10 0 0 10 0 10],node1,node2); % add pose constraint Z between node1 and node2. 
                    end;
                end;
                comptaLC=0;
                figure;
                show(position,'IDs','off');
                drawnow;
                title('Multi Robot Graph: Robust inter-session loop closings ')
                frame=getframe(theFigure);
                writeVideo(writerObj, frame);
                % optimize graph after the inter-session loop cosings have been added to it.
                position = optimizePoseGraph(position,'MaxIterations',150,'MaxTime', 1000, 'FunctionTolerance',1e-16,'VerboseOutput','on');
                optimized=true;
                fprintf('Optimizing Graph \n')
                figure;
                show(position,'IDs','off');
                drawnow;
                title('Optimized Multi Robot Graph')
            end;

        end;

    end;

close(writerObj)


   
    
return;

% numframeT1: num. of frames of trajectory 1
% Search external loops. 
function [externalMeasurements,position]=search_loops(minimum_ransac_inliers,position,pairs_loop_closings_image_nodes,numframeT1,globalData1, globalData2,externalMeasurements)
    % Initialize parameters and storage
   
    % Search loops with externally provided intersession loop closings detected with HALOC. 
    filteredloops_idx=globalData2.loopClosings(:,2) < numframeT1; %  index corresponding to rows of the loop closings file with a value of the second column lower
    % than the number of frames treated of the trajectory 1 . Keep those loops that include an image of trajectory 1 in the second column.
    loopsfiltered=globalData2.loopClosings(filteredloops_idx,:); % keep the loop closings corresponding to the selectd indexes. 
    candidateLoops=loopsfiltered((loopsfiltered(:,1)==globalData2.odoData(globalData2.curFrame).imNum),:); % take rows (1st and 2nd column) of loop closings filtered 
    % with a value of the first column equal than the image corresponding to the current frame of the second trajectory. 

    for j=1:size(candidateLoops,1)
        curLoop=candidateLoops(j,:); % store both columns
        i1=curLoop(1,1); % 1st image of the pair. It corresponds to trajectory 2
        % Search internal loops, externally provided (HALOC)
        imNum1 = [globalData2.odoData.imNum]; % imNum of the odoData
        tf1 = imNum1 == i1; % take the vector índex of imNums equal to i1; 

        descriptors1= globalData2.odoData(tf1).d; %  descriptors
        features1=globalData2.odoData(tf1).f; %  feature coordinates

        i2=curLoop(1,2); % image Trajectory 1
        imNum2 = [globalData1.odoData.imNum];
        tf2 = imNum2 == i2;
        descriptors2= globalData1.odoData(tf2).d;
        features2= globalData1.odoData(tf2).f;

        [matches,~]=vl_ubcmatch(descriptors1,descriptors2); % match features
        numinliers=curLoop(1,3); % retrieve the num of inliers

%% estimate motion with RANSAC
        [Z,~,best_consensus_set]=ransac_estimate_motion(features1(1:2,matches(1,:)),features2(1:2,matches(2,:)),globalData1.ransacIter,5,10,.75);
        loop.im1=i1 %image Trajectory 2
        loop.im2=i2 %  image Trajectory 1
        loop.transform = Z; % transform from the RANSAC process
        loop.inliers = numinliers; % number of inliers
        loop.matches = size(matches); % number of matches
        loop.ransac_inliers=size(best_consensus_set,2) % num. of inliers after RANSAC
        if loop.ransac_inliers> minimum_ransac_inliers % if the number of inliers after the RANSAC process is greater than the minimum, accumulate it. Else, discard it.
            externalMeasurements=[externalMeasurements; loop]; 
        end;
        image_pair_node1= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i1,:);
        image_pair_node2= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i2,:);
        if (~isempty(image_pair_node1)) &&  (~isempty(image_pair_node2))
            node1=image_pair_node1(1,2)  ;
            node2=image_pair_node2(1,2) ;
            addRelativePose(position,Z,[10 0 0 10 0 10],node1,node2); % add to the global graph the pose constraint corresponding to the inter-session loop closings, 
        end;
                
    end;
return;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PARAMETER INITIALIZATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define global parameters
% inicialization: 
function globalData=init(odoData,loopClosings,firstFrame,lastFrame)
    
    TwoSigmaPos=2;                      % position variance 
    TwoSigmaAngle=3*pi/180;             % orientation variance  
    % Odometry covariance
    globalData.Podo=[(TwoSigmaPos/2)^2,0,0;0,(TwoSigmaPos/2)^2,0;0,0,(TwoSigmaAngle/2)^2]; % covariance 3x3 diagonal matrix 
    % Loop closing covariance
    globalData.Ploop=globalData.Podo; 
    % Maximum distance to search loops
    globalData.loopMaxDistance=500;     % max. distance threshold to search local loops
    % Required loop closings to join maps
    globalData.loopsToJoin=7;          % minimum number of inter-session loops to join maps.  
    % Pre-computed loop closings
    globalData.loopClosings=loopClosings; 
    % Odometric information
    globalData.odoData=odoData;             
    % First and last frames to use
    globalData.firstFrame=firstFrame;        
    globalData.lastFrame=lastFrame;         
    % Current odometry frame
    globalData.curFrame=firstFrame;             
    % Number of iterations for RANSAC motion estimation
    globalData.ransacIter=100;
return;

%%%%%%%%%%%%
%%% SLAM %%%
%%%%%%%%%%%%

% Initialize SLAM data
function slamData=slam_init(slamPrev)
    % State vector. X(i*3-2:i*3) contains the roto-translation from frame
    % i-1 to frame i. X(1:3) is (0,0,0).
    slamData.X=[];              
    % State covariance
    slamData.P=[];              
    % Image number corresponding to frame i.
    slamData.F=[];              
    % SIFT features. theFeatures(i).f and theFeatures(i).d are the
    % coordinates and descriptors of the SIFT features of the image in
    % frame i.
    slamData.theFeatures=[];
    % Previous SLAM session
    slamData.slamPrev=slamPrev;
    % Indexes to the links between maps. Initialized to one to ease plot
    % when no links available.
    slamData.theLinks=1;
    % If 1, this session has still not been joined with the previous one.
    slamData.separatedMaps=1;
    % External loops, just for log and plot
    slamData.externalLoops=[];
    % Internal loops, just for log and plot
    slamData.internalLoops=[];
return;

% Initialize SLAM data. 
function slamData=slam_No_init(slamPrev,slamDataIN)
    % State vector. X(i*3-2:i*3) contains the roto-translation from frame
    % i-1 to frame i. X(1:3) is (0,0,0).
    slamData.X=slamDataIN.X;              
    % State covariance
    slamData.P=slamDataIN.P;              
    % Image number corresponding to frame i.
    slamData.F=slamDataIN.F;              
    % SIFT features. theFeatures(i).f and theFeatures(i).d are the
    % coordinates and descriptors of the SIFT features of the image in
    % frame i.
    slamData.theFeatures=slamDataIN.theFeatures;
    % Previous SLAM session
    slamData.slamPrev=slamPrev;
    % Indexes to the links between maps. Initialized to one to ease plot
    % when no links available.
    slamData.theLinks=slamDataIN.theLinks;
    % If 1, this session has still not been joined with the previous one.
    slamData.separatedMaps=slamDataIN.separatedMaps;
    % External loops, just for log and plot
    slamData.externalLoops=slamDataIN.externalLoops;
    % Internal loops, just for log and plot
    slamData.internalLoops=slamDataIN.internalLoops;
return;
% end

% Delete external loop storage; 
function [externalLoop,externalMeasurements]=external_loop_clean()
    externalLoop.F=[];
    externalLoop.Z=[];
    externalLoop.Fe=[];   
    externalMeasurements.F=[];     
    externalMeasurements.Fe=[];   
    externalMeasurements.Z=[];    
return;
%% acumulate the external Measurements
function [externalLoop,externalMeasurements]=external_loop_accumulate(externalMeasurementsIN,trajectory)
    externalLoop.F=[];
    externalLoop.Z=[];
    externalLoop.Fe=[];   
    externalMeasurements.F=externalMeasurementsIN.F;     
    externalMeasurements.Fe=externalMeasurementsIN.Fe;    
    externalMeasurements.Z=externalMeasurementsIN.Z;     
   
return;

% Augment SLAM state with current odometric estimate
function slamData=slam_augment_state(slamData,globalData)
    % Include current odometric estimate into state vector
    slamData.X=[slamData.X;globalData.odoData(globalData.curFrame).X]; % odometry of the current state 
   
    % Augment covariance matrix
    slamData.P(end+1:end+3,end+1:end+3)=globalData.Podo; 
    % Store image number
    slamData.F=[slamData.F globalData.odoData(globalData.curFrame).imNum]; 
    % Store features and descriptors
    slamData.theFeatures(end+1).f=globalData.odoData(globalData.curFrame).f;
    slamData.theFeatures(end).d=globalData.odoData(globalData.curFrame).d;
return;





% Accumulate external loops and decide if maps can be joined
function [externalLoop,joinMaps]=external_loop_update(slamData,globalData,externalLoop,externalMeasurements)
    % Accumulate external loops
    if slamData.separatedMaps && size(externalMeasurements.F,2)>0
        externalLoop.F=externalMeasurements.F;
        externalLoop.Z=externalMeasurements.Z;
        externalLoop.Fe=externalMeasurements.Fe;
    end;
    % State that maps can be joined if still sepparated and enough external
    % loops.
    joinMaps=slamData.separatedMaps && size(externalLoop.F,2)>=globalData.loopsToJoin;
return;
% end 
% this function adds the odometry data  with both sessions already joined. 
% global_position --> sorted state vector position --> global graph




function [global_position,position,lastnode] = general_position(slamData,lc)
    t = slamData.X;
    position= robotics.PoseGraph; % define the object "position" asa graph. A graph is a set of nodes that contain the global poses  
    
    
    for i=1:size(slamData.X)/3
        global_position(i).X = t(3*i-2:3*i); % global_position: state vector with odometric displacements
        [edge,edgeID]= addRelativePose(position,global_position(i).X,[10 0 0 10 0 10]); % creates a new pose node with a new edge constrain (global_position(i).X) 
        % from the last node
        % specified by relPose=global_position(i).X which now is the next element of the 
        % state vector. This is to connect it to the last node in the pose graph. [10 0 0 10 0 10] --> information Matrix=uncertainty
        % when adding a new new pose node, it calculates automatically and assigns the global pose to the node.
    end 


    % 425 is the total number of images in the dataset. Add all loops. This 425 is a little bit erratic, I know....
    if size(global_position,2) > 425 
       for i= 1: size(lc,1)
        addRelativePose(position,global_position(lc(i,1)).X,[10 0 0 10 0 10],lc(i,1),lc(i,2)); % creates a new pose node and uses an edge 
        % specified by relPose (global_position(lc(i,1)).X) to connect the node "lc(i,1)" with the node "lc(i,2)" in the pose graph. 
        % This new relative pose is an external loop closing. 
        % lc(i,1),lc(i,2) --> the relative pose (relPose) indicated in "global_position(lc(i,1)).X" which goes from origin node to destination node 
        % CARREFULL !!!!  global_position(lc(i,1)).X is not the transform, the transform is externalMeasurements.Z.
       end 
    end
    lastnode=edgeID;    
return ;

function lc=loops_closings(loopClosings,number_of_inliers_for_LC)
lc =[];
for i=1:size(loopClosings,1)
    if loopClosings(i,3)>number_of_inliers_for_LC
        lc = [lc ; loopClosings(i,:)];
    end
end
return;





% Join the two sessions. ->> when the main program of this source is executed, this function is not necessary anymore.
function slamData=slam_join(slamData,globalData,theMeasurements)
    % Join and augment external loops (logging only(
    slamData.externalLoops=[slamData.slamPrev.externalLoops slamData.externalLoops [theMeasurements.F;theMeasurements.Fe]];
%     % Optimization to compute the link
    [theLink,Plink]=compute_link(slamData,globalData,theMeasurements);
    % Join state and covariance through the link
    newX=[slamData.slamPrev.X;theLink;slamData.X]; % accumulate the SLAM of the previous session.
    newP=slamData.slamPrev.P;
    newP(end+1:end+3,end+1:end+3)=Plink;
    tmp=size(slamData.P,1);
    newP(end+1:end+tmp,end+1:end+tmp)=slamData.P;
    % Join frame ids. Place a -1 to the link, since it has no image.
    newF=[slamData.slamPrev.F -1 slamData.F];
    % Join features
    newFeatures=slamData.slamPrev.theFeatures;
    newFeatures(end+1).f=[];
    newFeatures(end+1:end+size(slamData.theFeatures,2))=slamData.theFeatures;
    % Store data
    slamData.X=newX;
    slamData.P=newP;
    slamData.F=newF;
    slamData.theFeatures=newFeatures;
    % Join links
     slamData.theLinks=[slamData.slamPrev.theLinks size(slamData.slamPrev.X,1)/3+1]; 
    % Join internal loops
     slamData.internalLoops=[slamData.slamPrev.internalLoops slamData.internalLoops]; 
    % State maps are no longer sepparated.
    slamData.separatedMaps=0;
    % Clean previous SLAM data
    slamData.slamPrev=[];
return;

% Draw trajectories and loops.
% Note that uncertainty ellipse is not correct since the robot covariance
% is not properly marginalized.
function draw_mslam(slamData)
    % Compute global poses
    X=zeros(3,1);
    P=zeros(3);
    Xh=zeros(3,size(slamData.X,1)/3);
    for i=1:size(slamData.X,1)/3
        [X,P]=compose_references(X,slamData.X(i*3-2:i*3,1),P,slamData.P(i*3-2:i*3,i*3-2:i*3));
        Xh(:,i)=X;
    end;
    
    % Draw internal loops
   %% if ~isempty(slamData.internalLoops)
   %%     L=[imnum2idx(slamData.internalLoops(1,:),slamData.F);imnum2idx(slamData.internalLoops(2,:),slamData.F)];
   %%     theX=[Xh(1,L(1,:));Xh(1,L(2,:))];
  %% %%     theY=[Xh(2,L(1,:));Xh(2,L(2,:))];
   %%     plot(theX,theY,'k');
   %%     hold on;
   %% end;

    % Draw external loops
    %%if ~isempty(slamData.externalLoops)
    %%    L=[imnum2idx(slamData.externalLoops(1,:),slamData.F);imnum2idx(slamData.externalLoops(2,:),slamData.F)];
    %%    theX=[Xh(1,L(1,:));Xh(1,L(2,:))];
    %%    theY=[Xh(2,L(1,:));Xh(2,L(2,:))];
    %%    plot(theX,theY,'r','LineWidth',2);
    %%    hold on;
    %%    plot(theX(1,:),theY(1,:),'ro');hold on;
    %%    plot(theX(2,:),theY(2,:),'ro');hold on;
    %%end;
    
    % Draw each trajectory
    for i=2:size(slamData.theLinks,2)
        plot(Xh(1,slamData.theLinks(i-1):slamData.theLinks(i)-1),Xh(2,slamData.theLinks(i-1):slamData.theLinks(i)-1),'LineWidth',2);
        %hold on;
    end;
    plot(Xh(1,slamData.theLinks(end):end),Xh(2,slamData.theLinks(end):end),'LineWidth',2);
    
    % Plot orientations at some points
    for i=1:10:size(Xh,2)
        draw_vehicle(Xh(:,i),50,'b');
        hold on;
    end;
    % Draw uncertainty ellipse (simple marginalization)
%     draw_ellipse(Xh(:,end),P,'r');
    % Draw current orientation
    draw_vehicle(Xh(:,end),100,'r');
    axis equal;
return;

% Move to next odometric data item
function globalData=next_frame(globalData)
    globalData.curFrame=globalData.curFrame+1;
return;





% Optimize trajectory (IEKF) according to internal loop closures.

function slamData=slam_update(slamData,globalData,theMeasurements)
    % Store internal loops (logging purposes)
    slamData.internalLoops=[slamData.internalLoops [theMeasurements.F;theMeasurements.Fe]];
    frameIndexes=imnum2idx(theMeasurements.F,slamData.F); % imágenes a número de indices 
    % Compute the predicted measurements (i.e. h in IEKF)
    predictedMeasurements=zeros(3,size(frameIndexes,2));
    for i=1:size(frameIndexes,2)
        X=zeros(3,1);
        for j=frameIndexes(:,i):(size(slamData.X,1)/3)-1,
            [X,~]=compose_references(X,slamData.X((j+1)*3-2:(j+1)*3,1),[],[]);
        end;
        predictedMeasurements(:,i)=X;
    end;
    % Iterate (IEKF)
    for j=1:10,  
        H=[];  
        theInnovation=zeros(size(theMeasurements.Z,2)*3,1);
        Rsm=zeros(size(theMeasurements.Z,2)*3);
        Xtmp=slamData.X;
        Ptmp=slamData.P;
        % For each measurement
        for i=1:size(theMeasurements.Z,2),
            % Compute partial Jacobian matrix
            [Htmp]=compute_observation_jacobian(Xtmp, predictedMeasurements(:,i), frameIndexes(:,i)+1);
            % Accumulate it
            H=[H;Htmp];
            % Prepare data
            theMeasurement=theMeasurements.Z(:,i);
            thePrediction=predictedMeasurements(:,i);
            theDifference=theMeasurement-thePrediction;
            theDifference(3,1)=normalize(theDifference(3,1));
            i1=i*3;
            i0=i1-2;
            theInnovation(i0:i1,1)=theDifference;
            Rsm(i0:i1,i0:i1)=globalData.Ploop;
        end;
        % IEKF update
        if (size(theMeasurements.Z,2)>0),
            tmp=slamData.P*H';
            Ptmp=slamData.P-tmp*(inv(H*tmp+Rsm))*H*slamData.P;
            Xtmp=Xtmp+Ptmp*H'*(inv(Rsm))*(theInnovation)-Ptmp/slamData.P*(Xtmp-slamData.X);              
        end;
    end;
    slamData.X=Xtmp;
    slamData.P=Ptmp;          
return;


%%%%%%%%%%%%%%%%%%%
%%% LOOP SEARCH %%%
%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%
%%% PLOT %%%
%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% OPTIMIZATION RELATED FUNCTIONS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimize the transformation from the end of previous map to the beginning
% of the current one by means of IEKF.
function [theLink,Plink]=compute_link(slamData,globalData,theMeasurements)
    % Compute initial estimate
    Z=theMeasurements.Z(:,1);
    [prePos,curPos]=compute_precur(slamData,theMeasurements,1);
    [X0,~]=invert_reference(prePos,[]);
    [X1,~]=invert_reference(curPos,[]);
    [theLink,~]=compose_references(X0,Z,[],[]);
    [theLink,~]=compose_references(theLink,X1,[],[]);
    Plink=globalData.Ploop;
    
    % Update ->> when the main program of this source is executed, this function is not necessary anymore.
%     nMeasurements=size(theMeasurements.Z,2);
%     Z=reshape(theMeasurements.Z,nMeasurements*3,1);
%     for j=1:100
%         h=[];
%         H=[];
%         Rsm=[];
%         for i=1:nMeasurements
%             [Xie,Xsj]=compute_precur(slamData,theMeasurements,i);
%             c1=cos(Xie(3));
%             s1=sin(Xie(3));
%             c2=cos(Xie(3)+theLink(3));
%             s2=sin(Xie(3)+theLink(3));
%             hij=[Xie(1)+theLink(1)*c1-theLink(2)*s1+Xsj(1)*c2-Xsj(2)*s2;
%                  Xie(2)+theLink(1)*s1+theLink(2)*c1+Xsj(1)*s2+Xsj(2)*c2;
%                  Xie(3)+theLink(3)+Xsj(3)];
%             Hij=[c1,-s1,-Xsj(2)*c2-Xsj(1)*s2;
%                  s1,c1,Xsj(1)*c2-Xsj(2)*s2;
%                  0,0,1];
%             h=[h;hij];
%             H=[H;Hij];
%             Rsm(end+1:end+3,end+1:end+3)=globalData.Ploop;
%         end;
% 
%         theInnovation=Z-h;
%         S=Rsm+H*Plink*H';
%         K=Plink*H'*inv(S);
%         theLink=theLink+K*theInnovation;
%         tmp=eye(3)-K*H;
%         Plink=tmp*Plink*tmp'+K*Rsm*K';
%     end;
return;

% Helper function for SLAM update. Computes the Jacobian matrix of the
% observation function.
function H=compute_observation_jacobian(Xslam,hk,startIndex)
  % Build the first part of the Jacobian, which is composed of zeros
  H=zeros(3,3*(startIndex-1));
  % The rest of items
  glk=zeros(3,1);
  % Initialize c=cos(0), s=sin(0)
  c=1;
  s=0;
  for i=startIndex:size(Xslam,1)/3,
    % Compute glk
    Xs=Xslam(i*3-2:i*3,1);
    glk=[glk(1)+Xs(1)*c-Xs(2)*s;glk(2)+Xs(1)*s+Xs(2)*c;glk(3)+Xs(3)];
    % Precompute some parameters
    c=cos(glk(3));
    s=sin(glk(3));
    p1=[-glk(1)*c-glk(2)*s+hk(1)*c+hk(2)*s;glk(1)*s-glk(2)*c-hk(1)*s+hk(2)*c];
    p2=glk(3)-Xslam(i*3,1);
    sp=sin(p2);
    cp=cos(p2);
    % Store the partial Jacobian into the output Jacobian matric
    H=[H [1,0,-p1(1)*s-p1(2)*c;0,1,p1(1)*c-p1(2)*s;0,0,1]*[cp,-sp,0;sp,cp,0;0,0,1]];    
  end;
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HELPER/AUXILIARY CODE %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





% Compute transformations from a given loop closure to the end of the
% previous map and from the beginning of the current map to the loop
% closure.
function [prePos,curPos]=compute_precur(slamData,theMeasurements,fIndex)
    F=imnum2idx(theMeasurements.F(fIndex),slamData.slamPrev.F);
    Fe=imnum2idx(theMeasurements.Fe(fIndex),slamData.F);
    curPos=compose_trajectory(slamData.X,1,Fe);
    prePos=compose_trajectory(slamData.slamPrev.X,F+1,size(slamData.slamPrev.X,1)/3);
return;

% Composes the tranaformations within a given interval of a state vector.
function X=compose_trajectory(Xslam,iStart,iEnd)
    X=zeros(3,1);
    for i=iStart:iEnd
        [X,~]=compose_references(X,Xslam(i*3-2:i*3,1),[],[]);
    end;
return;

% Converts from image number to index
function theIndexes=imnum2idx(imNum,Fslam)
    if isempty(imNum)
        theIndexes=[];
        return;
    end;
    theIndexes=[];
    for i=1:size(imNum,2)
        theIndexes=[theIndexes find(Fslam==imNum(i))];
    end;
    
return;

% Helper to check if last odometric data item has been reached.
function areThere=more_frames(globalData)
    areThere=globalData.curFrame<=globalData.lastFrame;
return;



% Draw a triangle pointing towards the current X axis.
function draw_vehicle (X,robotSize,theColor)
  vertices = [1.5 -1 -1 1.5
              0    1 -1  0 ]*robotSize/10;
  vertices = compose_point(X(1:3), vertices);
  plot(vertices(1,:), vertices(2,:), theColor);hold on;
  plot(X(1), X(2), 'r.');
return;

% Draw 2 sigma ellipse
function draw_ellipse(X,P,theColor)
    tita=linspace(0,2*pi,20);
    theCircle=[cos(tita);sin(tita)];
    [V,D]=eig(full(P(1:2,1:2)));
    theAxes=sqrt(9.2103*diag(D));
    tmp=(V*diag(theAxes))*theCircle;
    hp=line(tmp(1,:)+X(1),tmp(2,:)+X(2));
    set(hp,'Color',theColor);
    set(hp,'LineWidth',1.5);
return;











