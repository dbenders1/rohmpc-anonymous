function result = compute_check_obs_constraint_LMIs(do_print,do_check,model,c_o_sq,Xgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid;

  % Obtain model variables
  M = model.get_M();
  npos = size(M,1);

  % Make sure delta-RPI sublevel set is minimized to satisfy obstacle constraints
  i = 0;
  for j = 1:n_uxABXYgrid
    if n_uxABXYgrid == 1
      X = Xgrid;
    else
      X = Xgrid(:, :, j);
    end
    i = i + 1;
    ineq = [c_o_sq*eye(npos),M*X;
            (M*X)',X];

    if ~do_check
      result = [result;ineq>=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_obs_constraint_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      min_eig(i) = min(eig(ineq));
      if (min_eig(i) < -tol)
        result = [result;[i,min_eig(i)]];
        if do_print
          fprintf("[compute_check_obs_constraint_LMIs] Minimum eigenvalue of LMI %i: %f < -tol: %f!\n",i,min_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_obs_constraint_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_obs_constraint_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_obs_constraint_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
