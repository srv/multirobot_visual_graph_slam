for i=1:size(externalMeasurements,1)
	i1=externalMeasurements(i).im1;
	i2=externalMeasurements(i).im2;
 	image_pair_node1= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i1,:) %fbf 14/10/2019 
    image_pair_node2= pairs_loop_closings_image_nodes(pairs_loop_closings_image_nodes(:,1)==i2,:) %fbf 14/10/2019
    
    node1=image_pair_node1(1,2)  %fbf 14/10/2019
    node2=image_pair_node2(1,2) %fbf 14/10/2019
    Z=externalMeasurements(i).transform;
    addRelativePose(position,Z,[10 0 0 10 0 10],node1,node2);
end;
show(position,'IDs','off');


%position = optimizePoseGraph(position,'MaxIterations',150,'GradientTolerance',0.5e-12,'FunctionTolerance',1e-16,'InitialTrustRegionRadius',500,'StepTolerance',1e-4);

%position = optimizePoseGraph(position,'GradientTolerance',0.5e-12,'FunctionTolerance',1e-16,'InitialTrustRegionRadius',500,'StepTolerance',1e-4);
%position = optimizePoseGraph(position, 'g2o-levenberg-marquardt');

 %g2o parameters: 
position = optimizePoseGraph(position,'MaxIterations',150,'MaxTime', 1000, 'FunctionTolerance',1e-16,'VerboseOutput','on');

figure;
show(position,'IDs','off');
