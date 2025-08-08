function [data,timing] = compute_L_separate(do_print,solver_ops,do_check_sol,model,solver_tol,sol_check_tol,rho_o,P_delta)
  timer = tic;

  % Define decision variables
  L = sdpvar(model.nx,model.nx);
  L_norm_sq = sdpvar(1,1);

  % Define objective
  obj = L_norm_sq;

  % Compute LMIs
  timer_lmi = tic;
  do_check = false;
  options = 1;
  [n_uxABXYgrid,ugrid,xgrid,Agrid,~] = model.get_uxABXYgrid(do_print,do_check,options);
  P_delta_xu = full(P_delta(xgrid(:,1),ugrid(:,1)));
  con_contr = compute_check_observer_contraction_LMIs(do_print,do_check,model,rho_o,P_delta_xu,Agrid,L,n_uxABXYgrid,solver_tol);
  con_minnorm = compute_check_observer_minnorm_LMIs(do_print,do_check,model,P_delta_xu,L,L_norm_sq,solver_tol);
  con = [con_contr;con_minnorm];
  timing.t_L_separate_lmi = toc(timer_lmi);
  if do_print
    fprintf("[compute_L_separate] LMI computation time: %f\n",timing.t_L_separate_lmi);
  end

  % Optimize
  [~,timing.t_L_separate_opt] = optimize_wrapper(do_print,con,obj,solver_ops);

  % Obtain solution
  L = value(L);
  L_norm_sq = value(L_norm_sq);

  % Check solution
  timer_check_sol = tic;
  if do_check_sol
    do_check = true;
    compute_check_observer_contraction_LMIs(do_print,do_check,model,rho_o,P_delta_xu,Agrid,L,n_uxABXYgrid,sol_check_tol);
    compute_check_observer_minnorm_LMIs(do_print,do_check,model,P_delta_xu,L,L_norm_sq,sol_check_tol);
    timing.t_L_separate_check_sol = toc(timer_check_sol);
    if do_print
      fprintf("[compute_L_separate] Solution checking time: %f\n",timing.t_L_separate_check_sol);
    end
  end

  % Save relevant data to struct
  data.L = L;
  data.l = L_norm_sq;

  % Stop timing
  timing.t_L_separate_total = toc(timer);
  if do_print
    fprintf("[compute_L_separate] Total time: %f\n\n",timing.t_L_separate_total);
  end
end
