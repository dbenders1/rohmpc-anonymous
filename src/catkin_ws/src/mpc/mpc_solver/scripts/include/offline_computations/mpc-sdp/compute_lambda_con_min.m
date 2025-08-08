function lambda_con_min = compute_lambda_con_min(do_print,model)
  n_bytes = 0;

  % Obtain Q_mpc and R_mpc
  [Q_mpc,R_mpc] = model.get_QRmpc();

  % For each grid point:
  % Compute Jacobians
  % Construct closed-loop LQR law
  [n, ~, ~, Agrid, Bgrid] = model.get_uxABXYgrid(true,true,1);
  lambdas = zeros(size(Agrid,1),n);
  for i=1:n
    A = Agrid(:,:,i);
    B = Bgrid(:,:,i);
    [K,~,~] = lqr(A,B,Q_mpc,R_mpc);
    lambdas(:,i) = eig(A-B*K);
    if (any(real(lambdas(:,i)) >= 0))
      eig_idc = find(real(lambdas(:,i)) >= 0);
      for eig_idx = eig_idc
        cprintf('red',"[compute_lambda_con_min] Positive real eigenvalue detected: i=%i, eig_idx=%i\n",i,eig_idx);
      end
    end
    if do_print && rem(i,200) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_lambda_con_min] Evaluated closed-loop LQR controller eigenvalues %i/%i\n",i,n);
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_lambda_con_min] Evaluated closed-loop LQR controller eigenvalues %i/%i\n",i,n);
  end

  % Determine slowest eigenvalue over all grid points
  lambda_con_min = min(min(real(-lambdas)));
  if (lambda_con_min == inf)
    cprintf('red',"[compute_lambda_con_min] Computing upper bound on rho_c failed: slowest eigenvalue is inf! Returning -1.\n");
    lambda_con_min = -1;
  end
end
