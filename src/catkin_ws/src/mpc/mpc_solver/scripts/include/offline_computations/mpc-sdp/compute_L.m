function [data,timing] = compute_L(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,lambda_delta,lambda_epsilon,lambda_delta_epsilon,X_arr,Y_arr,W_bar,P_delta)
  timer_compute_L = tic;

  % Define decision variables
  L = sdpvar(model.nx,model.nx);
  delta_sq = sdpvar(1,1);
  eps_sq = sdpvar(1,1);
  H_bar = sdpvar(2*model.nx+1,2*model.nx+1);
  H1_bar = sdpvar(model.nx+1,model.nx+1);
  L_norm_sq = sdpvar(1,1);

  % Define objective
  % obj = L_norm_sq+lambda_delta_epsilon*eps_sq+lambda_delta*delta_sq;
  % obj = lambda_delta_epsilon*eps_sq+lambda_delta*delta_sq;
  % obj = 0.01*L_norm_sq+lambda_delta_epsilon*eps_sq+lambda_delta*delta_sq;
  obj = L_norm_sq+lambda_delta_epsilon*eps_sq+lambda_delta*delta_sq;

  % Compute LMIs
  timer_L_lmi = tic;
  do_check = false;
  [n_uxABXYgrid, ugrid, xgrid, Agrid, Bgrid, Xgrid, Ygrid, dotXgrid] = model.get_uxABXYgrid(do_print,do_check,2,X_arr,Y_arr);
  P_delta_xu = full(P_delta(xgrid(:,1),ugrid(:,1)));
  con_delta_rpi = compute_check_delta_RPI_LMIs(do_print,do_check,model,L,lambda_delta,lambda_delta_epsilon,delta_sq,eps_sq,H_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,solver_tol);
  con_eps_rpi = compute_check_epsilon_RPI_LMIs(do_print,do_check,model,L,lambda_epsilon,eps_sq,W_bar,H1_bar,Agrid,Xgrid,dotXgrid,n_uxABXYgrid,solver_tol);
  con_minnorm = compute_check_observer_minnorm_LMIs(do_print,do_check,model,P_delta_xu,L,L_norm_sq,solver_tol);
  con = [con_delta_rpi;con_eps_rpi;con_minnorm];
  % con = [con_delta_rpi;con_eps_rpi];
  timing.t_L_lmi = toc(timer_L_lmi);
  if do_print
    fprintf("[compute_L] LMI computation time: %f\n",timing.t_L_lmi);
  end

  % Optimize
  [~,timing.t_L_opt] = optimize_wrapper(do_print,con,obj,solver_ops);

  % Obtain solution
  obj = value(obj);
  L = value(L);
  delta_sq = value(delta_sq);
  eps_sq = value(eps_sq);
  H_bar = value(H_bar);
  H1_bar = value(H1_bar);
  L_norm_sq = value(L_norm_sq);

  % Check solution
  if do_check_sol
    timer_L_check_sol = tic;
    do_check = true;
    [n_uxABXYgrid, ~, ~, Agrid, Bgrid, Xgrid, Ygrid, dotXgrid] = model.get_uxABXYgrid(do_print,do_check,2,X_arr,Y_arr);
    compute_check_delta_RPI_LMIs(do_print,do_check,model,L,lambda_delta,lambda_delta_epsilon,delta_sq,eps_sq,H_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
    compute_check_epsilon_RPI_LMIs(do_print,do_check,model,L,lambda_epsilon,eps_sq,W_bar,H1_bar,Agrid,Xgrid,dotXgrid,n_uxABXYgrid,sol_check_tol);
    compute_check_observer_minnorm_LMIs(do_print,do_check,model,P_delta_xu,L,L_norm_sq,sol_check_tol);
    timing.t_L_check_sol = toc(timer_L_check_sol);
    if do_print
      fprintf("[compute_L] Solution checking time: %f\n",timing.t_L_check_sol);
    end
  end

  % Save relevant data to struct
  data.obj = obj;
  data.L = L;
  data.delta_sq = delta_sq;
  data.eps_sq = eps_sq;
  data.H_bar = H_bar;
  data.H1_bar = H1_bar;
  data.L_norm_sq = L_norm_sq;

  % Stop timing
  timing.t_L_total = toc(timer_compute_L);
  if do_print
    fprintf("[compute_L] Total time: %f\n\n",timing.t_L_total);
  end
end
