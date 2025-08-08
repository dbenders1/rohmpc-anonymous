%% Offline design
% Compute continuous-time terminal ingredients:
% - Incremental Lyapunov function V_delta = delta^T * P_delta * delta
% - Feedback law K_delta*delta
% - Terminal cost matrix P

% Initialize MATLAB interface
clear;
close all;
clc;

% Import necessary libraries
import casadi.*

% Start timing
timer = tic;

% Print information
do_print = true;

% Select which SDP to solve:
% - tmpc
% - rmpc
% - rompc
sdp_type = "rompc";

% Select whether to relative w bounds (non-zero w_bias) or absolute w
% bounds (w_bias=zeros)
use_w_rel = true;

% Select options for get_uxABXYgrid() function in model
get_grid_options = 0;

% Define mat filenames and store data from input file also in output file
input_mat_filepath = '../../../../../../mpc_model_id_mismatch/data/model_mismatch_results/falcon_t.mat';
output_mat_filepath = './offline_design.mat';
if exist(input_mat_filepath, 'file')
    load(input_mat_filepath);
else
    error('File does not exist at the specified path: %s. Ensure that the mpc_id_model_mismatch submodule is cloned', input_mat_filepath);
end
if ~use_w_rel
  w_bias = zeros(1,nw);
end
save(output_mat_filepath);

% Select model
model = FalconModelT(input_mat_filepath,use_w_rel);

% General optimization settings
if (sdp_type == "tmpc")
  solver_ops = sdpsettings('solver','sdpt3','verbose',1);
else
  solver_ops = sdpsettings('solver','mosek','verbose',1);
end
solver_tol = 1e-6;
do_check_sol = true;
sol_check_tol = solver_tol;


%% Describe system and obstacle avoidance constraints sets
% System (state and input) constraints
[~,~,~,~,L_s,~] = model.get_constraints();
L_s = L_s(2:2:end,:);

% Separate into lower and upper bounds of system constraints
sys_con_lb = model.get_lower_bounds();
sys_con_ub = model.get_upper_bounds();

% Determine gaps between lower and upper bounds
% NOTE: this is for scaling c_s_sq=c_s^2 in the cost, since otherwise the
% terminal set becomes very small for some inputs/states with respect to 
% other inputs/states
sys_con_m = (sys_con_ub+sys_con_lb)/2;
sys_con_halfs = sys_con_ub - sys_con_m;

% Define maximum obstacle distance
min_obs_dist = 0.1;


%% Controller design
% Estimate slowest eigenvalue of closed-loop LQR controller as rho_c upper bound
% lambda_con_min = compute_lambda_con_min(do_print,model);

% Controller optimization settings
rho_c = 1.3;
lambda_delta = 0.1;
delta = 1;
delta_sq = delta^2; % squared value of delta-sublevel set


%% Observer design
% Get observer gain
L = model.get_L(rho_c);

% Estimate slowest eigenvalue of closed-loop observer
lambda_obs_min = compute_lambda_obs_min(do_print,model,L);

% Check observer design
if (lambda_obs_min == -1)
    cprintf('red',"[main] Unstable observer: there exists eig((A-LC)') > 0 for one of the grid points! Exiting.\n");
    return;
end

% Observer optimization settings
small_tol = 1e-4;
lambda_ratio = lambda_delta/rho_c;
lambda_epsilon = lambda_ratio*lambda_obs_min;
l = compute_l(L);
lambda_delta_epsilon = l^2/(2*rho_c-lambda_delta)+small_tol;
eps_sq = lambda_delta*delta_sq/lambda_delta_epsilon-small_tol;
if (sdp_type == "rompc")
  epsilon = sqrt(eps_sq);
else
  epsilon = 0;
end

% Stop init timing
timing.t_init = toc(timer);
if do_print
  fprintf("[main] Init time: %f\n\n",timing.t_init);
end


%% Optimize for X, Y, c_s and c_o
[data_P_K_cs_co,timing_P_K_cs_co] = compute_Pdelta_Kdelta_cs_co(do_print,solver_ops,do_check_sol,sdp_type,model,solver_tol,sol_check_tol,L_s,sys_con_halfs,min_obs_dist,rho_c,lambda_delta,delta_sq,L,lambda_epsilon,lambda_delta_epsilon);
diagnostics_P_K_cs_co = data_P_K_cs_co.diagnostics;
X_arr = data_P_K_cs_co.X_arr;
Y_arr = data_P_K_cs_co.Y_arr;
P_delta = data_P_K_cs_co.P_delta;
K_delta = data_P_K_cs_co.K_delta;
c_s = data_P_K_cs_co.c_s;
if (sdp_type == "tmpc")
  [u,x] = model.get_uxhover();
  P = full(P_delta(x,u));
else
  c_o = data_P_K_cs_co.c_o;
  W_bar = data_P_K_cs_co.W_bar;
  if (sdp_type == "rompc")
    eps_sq = data_P_K_cs_co.eps_sq;
    epsilon = data_P_K_cs_co.epsilon;
    H_bar = data_P_K_cs_co.H_bar;
    H1_bar = data_P_K_cs_co.H1_bar;
  end
end


%% Optimize for L
% if (sdp_type == "rompc")
%   [data_L,timing_L] = compute_L(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,lambda_delta,lambda_epsilon,lambda_delta_epsilon,X_arr,Y_arr,W_bar,P_delta);
%   L = data_L.L;
%   delta_sq = data_L.delta_sq;
%   eps_sq = data_L.eps_sq;
%   delta = sqrt(delta);
%   epsilon = sqrt(eps_sq);
%   H_bar = data_L.H_bar;
%   H1_bar = data_L.H1_bar;
%   L_norm_sq = data_L.L_norm_sq;
% end


%% Check constraints feasibility if problem is feasible
timer_check_feasibility = tic;
% Compute alpha
if (sdp_type == "tmpc")
  % NOTE: can directly compute the 2-norm for matrix, since:
  % - c_o only depends on L_o, not on l_o, and L_o can be scaled so ||L||=1
  % - ||P^{-1/2} C' L'|| \leq ||P^{-1/2} C'|| ||L'|| \leq ||P^{-1/2} C'||
  M = model.get_M();
  c_o = norm(inv(sqrtm(P))\M');
  alpha = min_obs_dist/c_o;
  w_bar_c = 0;
  rho_c = 0;
elseif (sdp_type == "rmpc")
  [data_wbarc,timing_wbarc] = compute_w_bar_c(do_print,model,P_delta);
  w_bar_c = data_wbarc.w_bar_c;
  alpha = w_bar_c / rho_c;
elseif (sdp_type == "rompc")
  % [data_L,timing_L] = compute_L_separate(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,rho_c,P_delta);
  % L = data_L.L;
  % [data_epsilon,timing_epsilon] = compute_epsilon(do_print,model,P_delta,L);
  % epsilon = data_epsilon.epsilon;
  [data_wbarc,timing_wbarc] = compute_w_o_bar_c(do_print,model,L,epsilon,P_delta);
  w_bar_c = data_wbarc.w_bar_c;
  alpha = w_bar_c / rho_c + epsilon;
end

% Compute tightened constraint gaps
[sys_con_halfs_left_overs,obs_dist_left_over] = compute_tightened_constraint_gaps(sdp_type,model,sys_con_halfs,min_obs_dist,c_s,c_o,w_bar_c,rho_c,alpha);

% Print computed constants
if do_print
  if (sdp_type ~= "tmpc")
    fprintf("[main] rho_c: %f\n", rho_c);
    fprintf("[main] w_bar_c: %f\n", w_bar_c);
  end
  fprintf("[main] alpha: %f\n", alpha);
end

% Print left-over gaps in system constraints
sys_con_names = model.get_u_x_names();
if do_print
  fprintf("[main] %s\n", "Left-over of half of the system constraint region for:");
  for i=1:size(sys_con_names,1)
    fprintf("[main] %s:\t%3.4f / %3.4f\n", sys_con_names(i), sys_con_halfs_left_overs(i), sys_con_halfs(i));
  end

  % Print left-over gaps in obstacle avoidance constraints
  fprintf("[main] Left-over of the minimum obstacle distance:\t%3.4f / %3.4f\n", obs_dist_left_over(1), min_obs_dist);
end

% Stop timing
timing.t_check_feasibility = toc(timer_check_feasibility);
if do_print
  fprintf("[main] Feasibility checking time: %f\n\n",timing.t_check_feasibility);
end


%% Compute terminal cost matrix P
if (sdp_type ~= "tmpc")
  [data_P,timing_P] = compute_P(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,K_delta);
  P = data_P.P;
end


%% Obtain model-specific data for MPC solver generation
[data_mpc.Q,data_mpc.R] = model.get_QRmpc();
data_mpc.p_xidc = model.get_pxidc();


%% Stop total timing
timing.t_total = toc(timer);
if do_print
  fprintf("[main] Total time: %f\n\n",timing.t_total);
end


%% Merge timing structs
timing = merge_timing(timing,timing_P_K_cs_co);
if (sdp_type ~= "tmpc")
  % merge_timing(timing,timing_L);
  timing = merge_timing(timing,timing_wbarc);
  % timing = merge_timing(timing,timing_epsilon);
  timing = merge_timing(timing,timing_P);
end


%% Save data
save(output_mat_filepath,'sdp_type','model','do_print',...
     'solver_ops','solver_tol','do_check_sol','sol_check_tol',...
     'sys_con_lb','sys_con_ub','sys_con_halfs','min_obs_dist',...
     'rho_c','lambda_delta','delta','epsilon',...
     'sys_con_halfs_left_overs','obs_dist_left_over',...
     'timing',...
     '-append');

if (sdp_type == "rompc")
  save(output_mat_filepath,'L','lambda_epsilon','lambda_delta_epsilon',...
       '-append');
end


%% Merge data from other functions
save(output_mat_filepath,'-struct','data_P_K_cs_co','-append');
if (sdp_type ~= "tmpc")
  % save(output_mat_filename,'-struct','data_L','-append');
  save(output_mat_filepath,'-struct','data_wbarc','-append');
  % save(output_mat_filename,'-struct','data_data_epsilon','-append');
  save(output_mat_filepath,'-struct','data_P','-append');
end
save(output_mat_filepath,'-struct','data_mpc','-append');

