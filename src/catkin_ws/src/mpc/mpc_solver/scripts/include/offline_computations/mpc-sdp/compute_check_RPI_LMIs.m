function result = compute_check_RPI_LMIs(do_print,do_check,model,lambda_delta,W_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  nx = model.nx;
  [wgrid, n_wgrid] = model.get_wgrid();
  E = model.get_E();
  n_lmi = n_wgrid + n_uxABXYgrid;

  i = 0;
  for j=1:n_wgrid
    i = i + 1;
    w = wgrid(:, j);
    ineq = [zeros(nx,nx),E*w;
            (E*w)',zeros(1,1)] - W_bar;

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_RPI_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  for j=1:n_uxABXYgrid
    A_grid = Agrid(:, :, j);
    B_grid = Bgrid(:, :, j);
    if n_uxABXYgrid == 1
      X_grid = Xgrid;
      Y_grid = Ygrid;
      dotX_grid = dotXgrid;
    else
      X_grid = Xgrid(:, :, j);
      Y_grid = Ygrid(:, :, j);
      dotX_grid = dotXgrid(:, :, j);
    end
    i = i + 1;
    ineq = W_bar+...
           [(A_grid*X_grid+B_grid*Y_grid)+(A_grid*X_grid+B_grid*Y_grid)'-...
            dotX_grid+lambda_delta*X_grid,zeros(nx,1);...
            zeros(1,nx),-lambda_delta];

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_RPI_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
