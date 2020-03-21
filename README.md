# Multi Robot Visual Graph SLAM
==============
=== CREDIT ===
==============

2D MultiRobot Visual Graph SLAM. This approach is specially addressed to underwater or marine surface vehicles with cameras pointing downwards to the seabed. Althought not tested it could also be applied to terrestrial vehicles with a camera facing the ground. 

Non suitable for vehicles moving with 6 DoF.  

Author : Francisco Bonin Font, from part of the sources of Multi-session SLAM by Antoni Burguera
e-mail : francisco.bonin@uib.es, antoni.burguera@uib.es

A paper describing the theory behind this software has been submitted to a peer-reviewer journal. The proper reference will be added to this file after acceptance.

If you use this software, please cite that reference. If the reference is not available in this README, please contact us.

====================
=== REQUIREMENTS ===
====================

This program requires VLFeat to be properly installed. Please, visit:

http://www.vlfeat.org/

activate VLFEAT: run /home/xesc/RECERCA/VLFEAT/vlfeat-0.9.21/toolbox/vl_setup --> change this "/home/xesc/RECERCA/VLFEAT" for your directory where VLFEAT is installed

for instructions about how to download and properly install it.

=========================
=== BUILD THE PACKAGE ===
=========================

Download and unpack the package, if necessary, wherever you want. Open Matlab and navigate to the package source directory. Type "ls" or "dir" and check that several .m files, one .c and another .mexa64 files are there. 

The package is pure Matlab code except for one C file with MEX interface to Matlab. The mex file has already been build and provided with the sources. Its name is: least_squares_cartesian_mex.mexa64. In case of any error, just execute the file "install.m":

>> install, to rebuild it, but in principle, it is not necessary. Test everything before building the mex file.  

When compiling the mex file, Some warnings related to GCC version may appear, but no errors should occur. You should see "MEX completed successfully" appear.

In case of errors, check your Matlab MEX configuration by typing "help mex" and follow the instructions.

If the problem persists, a pure Matlab implementation of the MEX file is provided: least_squares_cartesian. Just search calls to least_squares_cartesian_mex within all provided Matlab sources and change them to least_squares_cartesian.

The package contains the sources, and the data to run a sample dataset: the loop closings file, the odometry of both sessions and the images. 
It is important to notice that the images have been named with consecutive numbers, and they are stored consecutively, that is, first all images of session 1, and the all images of session 2. The numeric name of the first image of trajectory 2 corresponds to the nummeric name of the last image of session 1, plus 1. For instance, in the case of the provided dataset, images from 1.png to 226.png correspond to trajectory 1, and images from 227.png to 425.png correspond to trajectory 2. And all must be in the same directory. This configuration is a must to run the code successfully, cause this image number is later associated to the graph node Id. 
Images have been resize to a considerable low resolution. The original resolution was 1920X1632 pixels, but this downsampling has been necessary to upload them into the github. If you want the original frames, just request them to: francisco.bonin@uib.es. 


====================
=== RUN THE CODE ===
====================

First of all, just to test the provided sources with the provided real marine dataset, unzip the contents of the provided compressed files "session2.zip" and "session2.zip", which contain the images of both sessions, in a single directory called "/data/images/ValldemossaGopro", which will be created in the directory where you put the Multi-robot matlab sources. In the directory "data" put the loopclosings.csv file and both odometry matlab  "odoData.mat" and "odoData2.mat" files also provided for this dataset in two zips called "images.zip" and "loopclosings_odometry.zip".

Within the source directory there is a "main.m" file. Execute it to see an example:

>> main

This example loads data from within the data directory. In the variables 

odoData1=load('data/odoData');
odoData2=load('data/odoData2');

The required data is odoData.mat and odoData2.mat, which hold preprocessed visual odometry as provided by https://github.com/aburguera/VISUAL_ODOMETRY_2D

It also loads loopclosings.csv. This CSV file must have three columns. The first and the second column hold the image numbers that close a loop. These numbers are matched against the imNum field of odoData. The third column is supposed to be a quality measure for each loop, though it is unused by this implementation:

loopClosings=load('data/loopclosings.csv'); 


Take into account: 
- parameters, such as :


	number_of_inliers_for_LC=80; % minimum number of inliers to consider an inter-session loop closing proposed by HALOC as valid. 
	loopsToJoin=1; %  minimum number of inter-session loops to joint maps 
	NofInterSessionLoopClosingsToOptimize=2; % number of inter-session loop closings accumulated to optimitze the global graph 

	have been set specifically for the example dataset provided in this repository. Set them according to your further datasets.


- imgPath='./data/images/ValldemossaGopro/'; % images gopro 1,2	: set here the directory where the images of your dataset are stored. This ./data refers to the directory "data" located in the same directory as the sources. For Windows users, use the corresponding syntaxis. 

- THe program writtes in video the matlab execution as a successive frames. Set the video name in: 
	writerObj = VideoWriter('video_multirobot.avi');

- Set the directory where the images of your dataset are stored in 
	imgPath='./data/images/ValldemossaGopro/'; % images gopro 1,2, just before the whole process starts. 

- In function mslam_video_multirobot(), uncomment %%writerObj = VideoWriter('video4.avi'); and %%writerObj.FrameRate = 2; to generate the video.


In this case, slamData1total is used to pass the previous map to the second session. Also, loopClosings is mandatory and must contain loops between the two maps. Otherwise, the maps won't be joined.

Navigate through the source code and read the comments and file headers to understand how the software works.

==================
=== PARAMETERS ===
==================

All the parameters are defined by the function

globalData=init(odoData,loopClosings,firstFrame,lastFrame)

Within this funcion, there are some parameters that you may need to modify:

* TwoSigmaPos and TwoSigmaAngle: The 2 sigma bounds for X, Y and Theta odometric estimates.
* loopsToJoin: How many external loops are accumulated before joining the maps.
* loopMaxDistance: The radius of the circle where loops are searched.

=======================
=== TROUBLESHOOTING ===
=======================

This software package has been tested using Matlab 8.5.0.197613 (R2015a) running on Ubuntu 16.04LTS with GCC 5.4.0., and also using Matlab R2018a (9.4.0.813654) running on Ubuntu 18.04 LTS.  Other configurations are untested.
