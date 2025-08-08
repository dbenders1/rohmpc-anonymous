function result = compute_check_contraction_LMIs(do_print,do_check,rho_c,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid;

  for i=1:n_lmi
    A_grid = Agrid(:, :, i);
    B_grid = Bgrid(:, :, i);
    if n_uxABXYgrid == 1
      X_grid = Xgrid;
      Y_grid = Ygrid;
      dotX_grid = dotXgrid;
    else
      X_grid = Xgrid(:, :, i);
      Y_grid = Ygrid(:, :, i);
      dotX_grid = dotXgrid(:, :, i);
    end
    ineq = (A_grid*X_grid+B_grid*Y_grid)+(A_grid*X_grid+B_grid*Y_grid)'-dotX_grid+2*rho_c*X_grid;

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_contraction_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_contraction_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_contraction_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_contraction_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_contraction_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
