function result = compute_check_descent_LMIs(do_print,do_check,Q,R,P,K_delta,ugrid,xgrid,Agrid,Bgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid;

  for i=1:n_lmi
    u_grid = ugrid(:, i);
    x_grid = xgrid(:, i);
    A_grid = Agrid(:, :, i);
    B_grid = Bgrid(:, :, i);
    K_delta_xu = full(K_delta(x_grid,u_grid));
    ineq = (A_grid+B_grid*K_delta_xu)'*P+P*(A_grid+B_grid*K_delta_xu)+Q+K_delta_xu'*R*K_delta_xu;

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_descent_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_descent_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_descent_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_descent_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_descent_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
