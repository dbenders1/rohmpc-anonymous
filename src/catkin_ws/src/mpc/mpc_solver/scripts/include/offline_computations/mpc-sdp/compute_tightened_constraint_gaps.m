function [sys_con_halfs_left_overs,obs_dist_left_over] = compute_tightened_constraint_gaps(sdp_type,model,sys_con_halfs,min_obs_dist,c_s,c_o,w_bar_c,rho_c,alpha)
  % Get system dimensions
  nu = model.get_nu();
  nx = model.get_nx();

  % Compute the left-overs of half of the gaps between lb and ub
  sys_con_halfs_left_overs = zeros(nu+nx,1);
  if (sdp_type ~= "rompc")
    sys_con_halfs_left_overs = sys_con_halfs-c_s*alpha;
  else
    sys_con_halfs_left_overs(1:nu) = sys_con_halfs(1:nu)-c_s(1:nu)*w_bar_c/rho_c;
    sys_con_halfs_left_overs(nu+1:end) = sys_con_halfs(nu+1:end)-c_s(nu+1:end)*alpha;
  end

  % Compute the left-over of the minimum obstacle distance
  if (sdp_type == "tmpc")
    obs_dist_left_over = min_obs_dist;
  else
    obs_dist_left_over = min_obs_dist-c_o*alpha;
  end
end
