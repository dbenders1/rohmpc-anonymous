function [data,timing] = compute_P(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,K_delta)
  timer_compute_P = tic;

  % Get system dimensions
  nu = model.get_nu();
  nx = model.get_nx();

  % Setting X_arr and Y_arr to zero since get_grid_options = 1
  get_grid_options = 1;
  X_arr = zeros(nx,nx);
  Y_arr = zeros(nu,nx);

  % Get model variables
  [Q_mpc,R_mpc] = model.get_QRmpc();

  % Define decision variables
  P = sdpvar(model.nx,model.nx);

  % Define objective
  obj = trace(P);

  % Compute LMIs
  timer_P_lmi = tic;
  do_check = false;
  [n_uxABXYgrid, ugrid, xgrid, Agrid, Bgrid] = model.get_uxABXYgrid(do_print,do_check,get_grid_options,X_arr,Y_arr);
  con_P = compute_check_P_LMIs(do_print,do_check,P,solver_tol);
  con_descent = compute_check_descent_LMIs(do_print,do_check,Q_mpc,R_mpc,P,K_delta,ugrid,xgrid,Agrid,Bgrid,n_uxABXYgrid,solver_tol);
  con = [con_P;con_descent];
  timing.t_P_lmi = toc(timer_P_lmi);
  if do_print
    fprintf("[compute_P] LMI computation time: %f\n",timing.t_P_lmi);
  end

  % Optimize
  [~,timing.t_P_opt] = optimize_wrapper(do_print,con,obj,solver_ops);

  % Obtain solution
  obj = value(obj);
  P = value(P);

  % Check solution
  if do_check_sol
    timer_P_check_sol = tic;
    do_check = true;
    [n_uxABXYgrid, ugrid, xgrid, Agrid, Bgrid] = model.get_uxABXYgrid(do_print,do_check,get_grid_options,X_arr,Y_arr);
    compute_check_P_LMIs(do_print,do_check,P,sol_check_tol);
    compute_check_descent_LMIs(do_print,do_check,Q_mpc,R_mpc,P,K_delta,ugrid,xgrid,Agrid,Bgrid,n_uxABXYgrid,sol_check_tol);
    timing.t_P_check_sol = toc(timer_P_check_sol);
    if do_print
      fprintf("[compute_P] Solution checking time: %f\n",timing.t_P_check_sol);
    end
  end

  % Save relevant data to struct
  data.obj = obj;
  data.P = P;

  % Stop timing
  timing.t_P_total = toc(timer_compute_P);
  if do_print
    fprintf("[compute_P] Total time: %f\n\n",timing.t_P_total);
  end
end
