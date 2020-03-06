% Name        : function [best_model, fail]=ransac_estimate_motion(S1, S2, ransac_iterations, num_random_samples, max_allowable_sample_error, min_points_in_consensus_multiplier)
% Description : Estimates planar 2D motion from points in S1 to points in
%               S2.
% Input       : S1         - Reference points. Each column has the X,Y
%                            coordinates.
%               S2         - Current points. Same format as S1. Each point
%                            in S2 corresponds to the point in S1 in the
%                            same index.
%               ransac_iterations - Number of times to iterate RANSAC.
%               num_random_samples - Number of random samples to build
%                            candidate motions.
%               max_allowable_sample_error - For a sample to be correct
%               (i.e. for a pair S1(:,i),S2(:,i) to be considered an
%               inlier) the distance after applying the candidate motion
%               must be below this value.
%               min_points_in_consensus_multiplier - For a candidate model
%               to be correct, the pairs S1(:,i),S2(:,i) that are
%               considered inliers must be larger than the number of points
%               in S1 or S2 times this multiplier.
% Output      : best_model - Motion (x,y,o)' from S1 to S2.
%               fail       - If zero, no problem. Otherwise, RANSAC
%               couldn't find a model and so best_model is meaningless.
% Author      : Antoni Burguera Burguera
%               antoni.burguera@uib.es
% Note        : Please, refer to the README.TXT file for information about
%               how to properly cite us if you use this software.
function [best_model, fail,best_consensus_set]=ransac_estimate_motion(S1, S2, ransac_iterations, num_random_samples, max_allowable_sample_error, min_points_in_consensus_multiplier),  
  %ransac_iterations=1000;
  %num_random_samples=3;
  %max_allowable_sample_error=0.2;
  %min_points_in_consensus_multiplier=0.6;
    
  best_error=Inf;
  best_model=zeros(3,1); % vector columna de dimensión 3
  best_consensus_set=[];
  for i=1:ransac_iterations, 
      % ciclo de 1 hasta el número de iteraciones 
      % maybe_inliers := n randomly selected values from data
      selected=zeros(1,size(S1,2));  % vector renglon de ceros de magnitud 1X
      nIters=1;
      while (sum(selected)<num_random_samples && nIters<1000),
        maybe_inliers=round((rand(1,num_random_samples)*(size(S1,2)-1))+1);
        selected(maybe_inliers)=1;
        nIters=nIters+1;
      end;
      % maybe_model := model parameters fitted to maybe_inliers
      maybe_model=least_squares_cartesian_mex(S1(:,maybe_inliers), S2(:, maybe_inliers));
      % consensus_set := maybe_inliers
      consensus_set=maybe_inliers;
      % for every point in data not in maybe_inliers 
      %   if point fits maybe_model with an error smaller than t
      %       add point to consensus_set      
      not_in_maybe_inliers=find(selected==0);
      SS2=compose_point(maybe_model, S2(:, not_in_maybe_inliers));
      tmp=SS2-S1(:,not_in_maybe_inliers);
      tmp=sqrt(sum(tmp.*tmp));
      points_to_add=not_in_maybe_inliers(tmp<max_allowable_sample_error);
            
      consensus_set=[consensus_set points_to_add];
      % if the number of elements in consensus_set is > d 
      %         (this implies that we may have found a good model,
      %         now test how good it is)      
      %size(consensus_set,2)/size(S1,2)
      if (size(consensus_set,2)>(size(S1,2)*min_points_in_consensus_multiplier)),
      % this_model := model parameters fitted to all points in consensus_set          
        this_model=least_squares_cartesian_mex(S1(:,consensus_set), S2(:,consensus_set));
      % this_error := a measure of how well this_model fits these points
        SS2=compose_point(this_model, S2(:,consensus_set));
        tmp=SS2-S1(:,consensus_set);
        this_error=sum(sqrt(sum(tmp.*tmp)));
      % if this_error < best_error
      %             (we have found a model which is better than any of the previous ones,
      %             keep it until a better one is found)
      %             best_model := this_model
      %             best_consensus_set := consensus_set
      %             best_error := this_error        
        if (this_error<best_error),
          best_model=this_model;
          best_consensus_set=consensus_set;
          best_error=this_error;
        end;        
      end;
    end;

    if (isempty(best_consensus_set))
        fail=1;
    else
        fail=0;
    end;   
return;