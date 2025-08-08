%% Offline design - SDP with LMIs to compute c_s, c_o, X, Y - bayesopt
% Compute quadratic incremetental Lyap for cont. time using LMIs
% Continous-time formulation
% Using Bayesian optimization

% Initialize MATLAB interface
clear;
close all;
clc;

% Import necessary libraries
import casadi.*

% Start timing
timer = tic;


%% Define Bayesian optimization problem
% Start timer
timer_bo = tic;

% Define hyperparameters
% Original values used:
% rho_c_init = 9.1331; % rho_c > 0
% lambda_init = 0.15792; % lambda > 0
rho_c_init = 1.3;
lambda_init = 0.1;
vars = [optimizableVariable('rho_c',[0.1,10],'Type','real'), ...
        optimizableVariable('lambda',[1e-6,0.2],'Type','real')];

% Define objective function
fun = @(x)runSDP(x);

% Solve Bayesian optimization problem
% NumCoupledConstraints is used to indicate if the problem is feasible
results = bayesopt(fun, ...
                   vars, ...
                   'InitialX', table(rho_c_init, lambda_init), ...
                   'IsObjectiveDeterministic',true, ...
                   'NumCoupledConstraints',1,...
                   'UseParallel',true, ...
                   'MaxObjectiveEvaluations',6);


%% Stop timing
timing.t_bo = toc(timer_bo);
timing.t_total = toc(timer);
fprintf("Total time: %f\n\n",timing.t_total);


%% Store results
rho_c = results.XAtMinObjective.rho_c;
lambda = results.XAtMinObjective.lambda;
save('bayesopt_rhoc_lambda.mat','rho_c','lambda','timing');


%% Define Bayesian optimization function
function [objective, constraint] = runSDP(vars)
  % Obtain hyperparameters
  rho_c = vars.rho_c;
  lambda = vars.lambda;
  fprintf("rho_c:%f, lambda:%f\n", rho_c, lambda);

  % Select model
  model = FalconModelT();

  % Maximum sublevel set size
  alpha = 1; % in terms of alpha in notes: alpha^2 (avoid ^2 in LMIs)
  % Note that in this case we do not use alpha^2 as we use the sqrt form of
  % V_delta

  % Check solution
  do_check_sol = true;
  solver_tol = 1e-6;

  % Print information
  do_print = false;

  % Describe system and obstacle avoidance constraints sets
  % System (state and input) constraints
  [~,~,~,~,L_s,~] = model.get_constraints();
  L_s = L_s(2:2:end,:);
  n_s = size(L_s,1);

  % Separate into lower and upper bounds of system constraints
  sys_con_lb = model.get_lower_bounds();
  sys_con_ub = model.get_upper_bounds();

  % Determine gaps between lower and upper bounds
  % NOTE: this is for scaling eps_s=c_s^2 in the cost, since otherwise the
  % terminal set becomes very small for some inputs/states with respect to 
  % other inputs/states
  sys_con_m = (sys_con_ub+sys_con_lb)/2;
  sys_con_halfs = sys_con_ub - sys_con_m;

  % Obstacle avoidance constraints
  % Get position selection matrix
  C = model.get_C();

  % Define maximum obstacle distance
  max_obs_dist = 0.5;

  % Set up optimization problem (everything independent of hyperparam)
  % Define decision variables
  eps_s = sdpvar(n_s,1); % in terms of eps in notes: eps^2 (avoid ^2 in LMIs)
  eps_o = sdpvar(1,1);   % in terms of eps in notes: eps^2 (avoid ^2 in LMIs)
  W_bar = sdpvar(model.nx+1,model.nx+1); % to reduce computational complexity of RPI LMIs
  np = 0;
  X_arr = sdpvar(model.nx,model.nx);
  Y_arr = sdpvar(model.nu,model.nx);
  % np = model.get_np();
  % X_arr = sdpvar(model.nx,model.nx,np+1);
  % Y_arr = sdpvar(model.nu,model.nx,np+1);

  % Define objective
  % obj = -log(det(X))
  % obj = sum(eps_s)+sum(eps_o);
  obj = sum(eps_s./sys_con_halfs.^2)+eps_o/max_obs_dist^2;

  % Define solver options
  ops = sdpsettings('solver','mosek','verbose',1);

  % Define LMIs
  do_check = false;
  [~, ~, Agrid, Bgrid, Xgrid, Ygrid, n_uxABXYgrid] = model.get_uxABXYgrid(do_print,do_check,X_arr,Y_arr);
  con_X = compute_check_X_LMIs(do_print,do_check,X_arr,solver_tol);
  con_sys = compute_check_sys_constraint_LMIs(do_print,do_check,n_s,L_s,alpha,eps_s,Xgrid,Ygrid,n_uxABXYgrid,solver_tol);
  con_obs = compute_check_obs_constraint_LMIs(do_print,do_check,C,alpha,eps_o,Xgrid,n_uxABXYgrid,solver_tol);
  con_contr = compute_check_contraction_LMIs(do_print,do_check,rho_c,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,solver_tol);
  con_rpi = compute_check_RPI_LMIs(do_print,do_check,model,lambda,alpha,W_bar,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,solver_tol);
  con = [con_X;con_sys;con_obs;con_contr;con_rpi];

  % Optimize
  diagnostics = optimize(con,obj,ops);
  if diagnostics.problem == 0
    constraint = -1;
  else
    constraint = 1;
  end

  % Obtain solution
  eps_s = value(eps_s);
  eps_o = value(eps_o);
  c_s = sqrt(eps_s);
  c_o = sqrt(eps_o);
  W_bar = value(W_bar);
  X_arr = value(X_arr);
  Y_arr = value(Y_arr);
  [P_delta, K_delta] = model.get_PKfun(X_arr, Y_arr); % NOTE: this P_delta is in squared form!

  % Check solution
  if do_check_sol
    do_check = true;
    [~, ~, Agrid, Bgrid, Xgrid, Ygrid, n_uxABXYgrid] = model.get_uxABXYgrid(do_print,do_check,X_arr,Y_arr);
    if ~isempty(compute_check_X_LMIs(do_print,do_check,X_arr,solver_tol))
      cprintf([1,0,0], "(rho_c:%f,lambda:%f) X LMI violation!\n",rho_c,lambda);
    end
    if ~isempty(compute_check_sys_constraint_LMIs(do_print,do_check,n_s,L_s,alpha,eps_s,Xgrid,Ygrid,n_uxABXYgrid,solver_tol))
      cprintf([1,0,0], "(rho_c:%f,lambda:%f) sys LMI violation!\n",rho_c,lambda);
    end
    if ~isempty(compute_check_obs_constraint_LMIs(do_print,do_check,C,alpha,eps_o,Xgrid,n_uxABXYgrid,solver_tol))
      cprintf([1,0,0], "(rho_c:%f,lambda:%f) obs LMI violation!\n",rho_c,lambda);
    end
    if ~isempty(compute_check_contraction_LMIs(do_print,do_check,rho_c,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,solver_tol))
      cprintf([1,0,0], "(rho_c:%f,lambda:%f) contraction LMI violation!\n",rho_c,lambda);
    end
    if ~isempty(compute_check_RPI_LMIs(do_print,do_check,model,lambda,alpha,W_bar,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,solver_tol))
      cprintf([1,0,0], "(rho_c:%f,lambda:%f) RPI LMI violation!\n",rho_c,lambda);
    end
  end

  % Compute constraint tightening
  [~,~,w_bar_c,~] = compute_rho_wbar_beta(do_print,model,rho_c,P_delta,K_delta);
  [sys_con_halfs_left_overs,obs_dist_left_over] = compute_tightened_constraint_gaps(sys_con_halfs,max_obs_dist,c_s,c_o,rho_c,w_bar_c);
  if any(sys_con_halfs_left_overs < 0)
    cprintf([1,0,0], "(rho_c:%f,lambda:%f) sys con too tight!\n",rho_c,lambda);
  end
  if any(obs_dist_left_over < 0)
    cprintf([1,0,0], "(rho_c:%f,lambda:%f) obs con too tight!\n",rho_c,lambda);
  end

  % Compute objective value
  objective = -min([sys_con_halfs_left_overs;obs_dist_left_over]);
end
