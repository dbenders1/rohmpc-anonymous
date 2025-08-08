function lambda_obs_min = compute_lambda_obs_min(do_print,model,L)
  n_bytes = 0;
  C = model.get_C();

  % For each grid point:
  % Compute Jacobians
  % Construct closed-loop LQR law
  [n, ~, ~, Agrid, ~] = model.get_uxABXYgrid(true,true,1);
  lambdas = zeros(size(Agrid,1),n);
  for i=1:n
    lambdas(:,i) = eig((Agrid(:,:,i)-L*C)');
    if (any(real(lambdas(:,i)) >= 0))
      eig_idc = find(real(lambdas(:,i)) >= 0);
      for eig_idx = eig_idc
        cprintf('red',"[compute_lambda_obs_min] Positive real eigenvalue detected: i=%i, eig_idx=%i\n",i,eig_idx);
      end
    end
    if do_print && rem(i,200) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_lambda_obs_min] Evaluated closed-loop observer eigenvalues %i/%i\n",i,n);
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_lambda_obs_min] Evaluated closed-loop observer eigenvalues %i/%i\n",i,n);
  end

  % Determine slowest eigenvalue over all grid points
  lambda_obs_min = min(min(real(-lambdas)));
  if (lambda_obs_min == inf)
    cprintf('red',"[compute_lambda_obs_min] Computing upper bound on rho_c failed: slowest eigenvalue is inf! Returning -1.\n");
    lambda_obs_min = -1;
  end
end
