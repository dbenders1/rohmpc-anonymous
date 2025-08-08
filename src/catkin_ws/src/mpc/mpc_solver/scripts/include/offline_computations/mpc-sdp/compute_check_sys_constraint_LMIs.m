function result = compute_check_sys_constraint_LMIs(do_print,do_check,n_s,L_s,c_s_sq,Xgrid,Ygrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid * n_s;

  % Initialize result with no LMIs being violated, fill below if violated
  if do_check
    result = zeros(0,2);
  end

  % Make sure delta-RPI sublevel set is minimized to satisfy system constraints
  i = 0;
  for j = 1:n_uxABXYgrid
    if n_uxABXYgrid == 1
      X = Xgrid;
      Y = Ygrid;
    else
      X = Xgrid(:, :, j);
      Y = Ygrid(:, :, j);
    end
    for k = 1:n_s
      i = i + 1;
      ineq = [c_s_sq(k),L_s(k,:)*[Y;X];
              (L_s(k,:)*[Y;X])',X];

      if ~do_check
        result = [result;ineq>=0];
        if do_print && rem(i,200) == 0
          fprintf(repmat('\b',1,n_bytes));
          n_bytes = fprintf("[compute_check_sys_constraint_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
        end
      else
        min_eig(i) = min(eig(ineq));
        if (min_eig(i) < -tol)
          result = [result;[i,min_eig(i)]];
          if do_print
            fprintf("[compute_check_sys_constraint_LMIs] Minimum eigenvalue of LMI %i: %f < -tol: %f!\n",i,min_eig(i),tol);
          end
        end
        if do_print && rem(i,200) == 0
          fprintf(repmat('\b',1,n_bytes));
          n_bytes = fprintf("[compute_check_sys_constraint_LMIs] Checked LMI %i/%i\n",i,n_lmi);
        end
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_sys_constraint_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_sys_constraint_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
