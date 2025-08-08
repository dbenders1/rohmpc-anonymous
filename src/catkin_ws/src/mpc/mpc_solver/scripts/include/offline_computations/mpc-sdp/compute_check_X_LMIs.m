function result = compute_check_X_LMIs(do_print,do_check,model,Xgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  nx = model.nx;
  n_lmi = n_uxABXYgrid;

  i = 0;
  for j = 1:n_lmi
    if n_uxABXYgrid == 1
      X_grid = Xgrid;
    else
      X_grid = Xgrid(:, :, j);
    end
    i = i + 1;
    ineq = X_grid - tol*eye(nx);
    if ~do_check
      result = [result;ineq>=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_X_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      min_eig(i) = min(eig(ineq));
      if (min_eig(i) < -tol)
        result = [result;[i,min_eig(i)]];
        if do_print
          fprintf("[compute_check_X_LMIs] Minimum eigenvalue of LMI %i: %f < -tol: %f!\n",i,min_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_X_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_X_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_X_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
