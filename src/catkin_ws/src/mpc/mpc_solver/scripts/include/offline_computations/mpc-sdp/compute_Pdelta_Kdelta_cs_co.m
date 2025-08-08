function [data,timing] = compute_Pdelta_Kdelta_cs_co(do_print,solver_ops,do_check_sol,sdp_type,model,solver_tol,sol_check_tol,L_s,sys_con_halfs,min_obs_dist,rho_c,lambda_delta,delta_sq,L,lambda_epsilon,lambda_delta_epsilon)
  timer = tic;

  % Get controller information
  if (sdp_type == "tmpc")
    [Q,R] = model.get_QRmpc();
    Q_eps = sqrtm(Q);
    R_eps = sqrtm(R);
  end

  % Compute H_bar, W_bar and H1_bar
  % h = lambda_delta*delta_sq-lambda_delta_epsilon*eps_sq-small_tol;
  % w = lambda_epsilon*eps_sq-h-small_tol;
  % [~,H_bar] = compute_H_bar(do_print,model,L,h,solver_tol);
  % [~,W_bar] = compute_W_bar(do_print,model,w,solver_tol);
  % [~,H1_bar] = compute_H1_bar(do_print,model,L,h,solver_tol);

  % Check optimization settings
  if (sdp_type == "rompc")
    l = compute_l(L);
    if ((-2*rho_c+lambda_delta)*(-lambda_delta_epsilon)-l^2 < 0)
      cprintf('red','[compute_Pdelta_Kdelta_cs_co] Please ensure that (-2*rho_c+lambda_delta)*(-lambda_delta_epsilon)-l^2 >= 0! Exiting.\n');
      return;
    % elseif (lambda_delta*delta_sq < lambda_delta_epsilon*eps_sq)
    %   cprintf('red','[compute_Pdelta_Kdelta_cs_co] Please ensure that lambda_delta*delta_sq >= lambda_delta_epsilon*eps_sq! Exiting.\n');
    %   return;
    % elseif (h/w < 1e-3 || h/w > 1e3)
    %   cprintf('red','[compute_Pdelta_Kdelta_cs_co] Ratio between h and w too large. Please ensure that -1e-3 <= h/w <= 1e-3! Exiting.\n');
    %   return;
    % elseif (h+lambda_delta_epsilon*eps_sq-lambda_delta*delta_sq > 0)
    %   cprintf('red',"[compute_Pdelta_Kdelta_cs_co] Please ensure that H_bar(end,end)+lambda_delta_epsilon*eps_sq-lambda_delta*delta_sq <= 0! Exiting.\n");
    %   return;
    % elseif (w+h-lambda_epsilon*eps_sq > 0)
    %   cprintf('red',"[compute_Pdelta_Kdelta_cs_co] w+h-lambda_epsilon*eps_sq <= 0! Exiting.\n");
    %   return;
    end
  end

  % Define decision variables
  n_s = size(L_s,1);
  c_s_sq = sdpvar(n_s,1);                     % system constraints tightening constants in squared form
  if (sdp_type ~= "tmpc")
    c_o_sq = sdpvar(1,1);                       % obstacle avoidance constraints tightening constant in squared form
    W_bar = sdpvar(model.nx+1,model.nx+1);      % to reduce computational complexity of epsilon-RPI LMIs
    if (sdp_type == "rompc")
      eps_sq = sdpvar(1,1);                       % observer error invariant set in squared form
      H_bar = sdpvar(2*model.nx+1,2*model.nx+1);  % to reduce computational complexity of delta-RPI LMIs
      H1_bar = sdpvar(model.nx+1,model.nx+1);     % to reduce computational complexity of epsilon-RPI LMIs
    end
  end
  [nX, nY] = model.get_nXnY();
  if nX == 1
    X_arr = sdpvar(model.nx,model.nx);
  else
    X_arr = sdpvar(model.nx,model.nx,nX);
  end
  if nY == 1
    Y_arr = sdpvar(model.nu,model.nx);
  else
    Y_arr = sdpvar(model.nu,model.nx,nY);
  end
  if nX > 0 || nY > 0
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] Parameter implementation does not take care of dependency on u, only on x!\n');
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] X_dot computation for parameters equal to states affected by Ew should involve Ew term!\n');
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] Also: make sure X_dot computation is changed suitably when using E(x)!\n');
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] Also: make sure that w_bar_c and w_o_bar_c computations are changed suitably => using P_delta(x) instead of constant P_delta!\n');
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] Also: make sure that observer gain L computation is adjusted if using E(x)\n');
    cprintf('red','[compute_Pdelta_Kdelta_cs_co] Also: make sure that terminal cost matrix P LMI is adjusted if using E(x)\n');
  end
  if (sdp_type == "tmpc" && (nX > 1 || nY > 1))
    exit("The tracking SDP does currently not support nX > 1 or nY > 1");
  end

  % Define objective
  if (sdp_type == "tmpc")
    % NOTE:
    % - Infeasible for (c <= 0 (unbounded), c >= 1e7 (numerical issues))
    % - Penalize constraints tightening to decrease terminal set size
    % - Normalize constraints w.r.t. their constraints interval to tighten each
    % constraint equally (as a percentage of the complete range)
    % - c large => reshape P to achieve equal tightening in every dimension
    % - Tune c, starting from a low value until the input tightening saturates
    c = 20;
    obj = -log(det(X_arr)) + c*sum(c_s_sq./sys_con_halfs.^2);
  elseif (sdp_type == "rmpc")
    obj = sum(c_s_sq./sys_con_halfs.^2)+c_o_sq/min_obs_dist^2;
  elseif (sdp_type == "rompc")
    % obj = sum(c_s_sq(1:model.nu)./sys_con_halfs(1:model.nu).^2)+...
    %       (1+eps_sq)*sum(c_s_sq(model.nu+1:end)./sys_con_halfs(model.nu+1:end).^2)+...
    %       (1+eps_sq)*c_o_sq/min_obs_dist^2;
    obj = sum(c_s_sq(1:model.nu)./sys_con_halfs(1:model.nu).^2)+...
          sum(c_s_sq(model.nu+1:end)./sys_con_halfs(model.nu+1:end).^2)+...
          c_o_sq/min_obs_dist^2+...
          2e3*eps_sq;
  end

  % Compute LMIs
  timer_lmi = tic;
  do_check = false;
  get_grid_options = 2;
  [n_uxABXYgrid, ~, ~, Agrid, Bgrid, Xgrid, Ygrid, dotXgrid] = model.get_uxABXYgrid(do_print,do_check,get_grid_options,X_arr,Y_arr);
  con_X = compute_check_X_LMIs(do_print,do_check,model,Xgrid,n_uxABXYgrid,solver_tol);
  con_sys = compute_check_sys_constraint_LMIs(do_print,do_check,n_s,L_s,c_s_sq,Xgrid,Ygrid,n_uxABXYgrid,solver_tol);
  if (sdp_type == "tmpc")
    con_track = compute_check_tracking_LMIs(do_print,do_check,model,Q_eps,R_eps,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,solver_tol);
    con = [con_X;con_sys;con_track];
  else
    con_obs = compute_check_obs_constraint_LMIs(do_print,do_check,model,c_o_sq,Xgrid,n_uxABXYgrid,solver_tol);
    con_contr = compute_check_contraction_LMIs(do_print,do_check,rho_c,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,solver_tol);
    if (sdp_type == "rmpc")
      con_delta_rpi = compute_check_RPI_LMIs(do_print,do_check,model,lambda_delta,W_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,solver_tol);
      con = [con_X;con_sys;con_obs;con_contr;con_delta_rpi];
    elseif (sdp_type == "rompc")
      con_delta_rpi = compute_check_delta_RPI_LMIs(do_print,do_check,model,L,lambda_delta,lambda_delta_epsilon,delta_sq,eps_sq,H_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,solver_tol);
      con_eps_rpi = compute_check_epsilon_RPI_LMIs(do_print,do_check,model,L,lambda_epsilon,eps_sq,W_bar,H1_bar,Agrid,Xgrid,dotXgrid,n_uxABXYgrid,solver_tol);
      con = [con_X;con_sys;con_obs;con_contr;con_delta_rpi;con_eps_rpi];
    end
  end
  timing.t_P_K_cs_co_lmi = toc(timer_lmi);
  if do_print
    fprintf("[compute_Pdelta_Kdelta_cs_co] LMI computation time: %f\n",timing.t_P_K_cs_co_lmi);
  end

  % Optimize
  [diagnostics,timing.t_P_K_cs_co_opt] = optimize_wrapper(do_print,con,obj,solver_ops);

  % Obtain solution
  obj = value(obj);
  X_arr = value(X_arr);
  Y_arr = value(Y_arr);
  c_s_sq = value(c_s_sq);
  c_s = sqrt(c_s_sq);
  [P_delta, K_delta] = model.get_PKfun(X_arr, Y_arr); % NOTE: this P_delta is in squared form!
  if (sdp_type ~= "tmpc")
    c_o_sq = value(c_o_sq);
    c_o = sqrt(c_o_sq);
    W_bar = value(W_bar);
    if (sdp_type == "rompc")
      eps_sq = value(eps_sq);
      epsilon = sqrt(eps_sq);
      H_bar = value(H_bar);
      H1_bar = value(H1_bar);
    end
  end

  % Check solution
  timer_check_sol = tic;
  if do_check_sol
    do_check = true;
    [n_uxABXYgrid, ugrid, xgrid, Agrid, Bgrid, Xgrid, Ygrid, dotXgrid] = model.get_uxABXYgrid(do_print,do_check,2,X_arr,Y_arr);

    % Check LMI satisfaction
    compute_check_X_LMIs(do_print,do_check,model,Xgrid,n_uxABXYgrid,sol_check_tol);
    compute_check_sys_constraint_LMIs(do_print,do_check,n_s,L_s,c_s_sq,Xgrid,Ygrid,n_uxABXYgrid,sol_check_tol);
    if (sdp_type == "tmpc")
      compute_check_tracking_LMIs(do_print,do_check,model,Q_eps,R_eps,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,sol_check_tol);
    else
      compute_check_obs_constraint_LMIs(do_print,do_check,model,c_o_sq,Xgrid,n_uxABXYgrid,sol_check_tol);
      compute_check_contraction_LMIs(do_print,do_check,rho_c,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
      if (sdp_type == "rmpc")
        compute_check_RPI_LMIs(do_print,do_check,model,lambda_delta,W_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
      elseif (sdp_type == "rompc")
        compute_check_delta_RPI_LMIs(do_print,do_check,model,L,lambda_delta,lambda_delta_epsilon,delta_sq,eps_sq,H_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
        compute_check_epsilon_RPI_LMIs(do_print,do_check,model,L,lambda_epsilon,eps_sq,W_bar,H1_bar,Agrid,Xgrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
      end
    end

    % Check condition numbers and eigenvalues of P_delta(x,u)
    condition_numbers = zeros(n_uxABXYgrid,1);
    eig_vals = zeros(model.nx,n_uxABXYgrid);
    for i = 1:n_uxABXYgrid
      u = ugrid(:,i);
      x = xgrid(:,i);
      condition_numbers(i) = cond(full(P_delta(x,u)));
      eig_vals(:,i) = eig(full(P_delta(x,u)));
    end
    if do_print
      % fprintf("cond(P): %e\n", condition_numbers);
      fprintf("[compute_Pdelta_Kdelta_cs_co] Max eig(P): %e\n", max(eig_vals,[],"all"));
    end
  end
  timing.t_P_K_cs_co_check_sol = toc(timer_check_sol);
  if do_print
    fprintf("[compute_Pdelta_Kdelta_cs_co] Solution checking time: %f\n",timing.t_P_K_cs_co_check_sol);
  end

  % Save relevant data to struct
  data.diagnostics = diagnostics;
  data.obj = obj;
  data.X_arr = X_arr;
  data.Y_arr = Y_arr;
  data.P_delta = P_delta;
  data.K_delta = K_delta;
  data.c_s_sq = c_s_sq;
  data.c_s = c_s;
  if (sdp_type ~= "tmpc")
    data.c_o_sq = c_o_sq;
    data.c_o = c_o;
    data.W_bar = W_bar;
    if (sdp_type == "rompc")
      data.eps_sq = eps_sq;
      data.epsilon = epsilon;
      data.H_bar = H_bar;
      data.H1_bar = H1_bar;
    end
  end

  % Stop timing
  timing.t_P_K_cs_co_total = toc(timer);
  if do_print
    fprintf("[compute_Pdelta_Kdelta_cs_co] Total time: %f\n\n",timing.t_P_K_cs_co_total);
  end
end
