function result = compute_check_tracking_LMIs(do_print,do_check,model,Q_eps,R_eps,Agrid,Bgrid,Xgrid,Ygrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid;
  nx = model.get_nx();
  nu = model.get_nu();

  % Initialize result with no LMIs being violated, fill below if violated
  if do_check
    result = zeros(0,2);
  end

  i = 0;
  for j = 1:n_uxABXYgrid
    if n_uxABXYgrid == 1
      A = Agrid;
      B = Bgrid;
      X = Xgrid;
      Y = Ygrid;
    else
      A = Agrid(:, :, j);
      B = Bgrid(:, :, j);
      X = Xgrid(:, :, j);
      Y = Ygrid(:, :, j);
    end
    i = i + 1;
    ineq = [A*X+B*Y+(A*X+B*Y)',(Q_eps*X)',(R_eps*Y)';
            Q_eps*X,-eye(nx),zeros(nx,nu);
            R_eps*Y,zeros(nu,nx),-eye(nu)];

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_tracking_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_tracking_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_tracking_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_tracking_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_tracking_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
