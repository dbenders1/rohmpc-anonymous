function result = compute_check_delta_RPI_LMIs(do_print,do_check,model,L,lambda_delta,lambda_delta_epsilon,delta_sq,eps_sq,H_bar,Agrid,Bgrid,Xgrid,Ygrid,dotXgrid,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  nx = model.nx;
  [etagrid, n_etagrid] = model.get_etagrid();
  C = model.get_C();
  F = model.get_F();
  n_lmi = n_etagrid + n_uxABXYgrid;
  % n_lmi = n_uxABXYgrid;

  i = 0;
  for j=1:n_etagrid
    i = i + 1;
    eta = etagrid(:, j);
    ineq = [zeros(nx,nx),zeros(nx,nx),L*F*eta;
            zeros(nx,nx),zeros(nx,nx),zeros(nx,1);
            (L*F*eta)',zeros(1,nx),0] - H_bar;

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_delta_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_delta_RPI_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_delta_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
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
    ineq = H_bar+...
           [(A_grid*X_grid+B_grid*Y_grid)+(A_grid*X_grid+B_grid*Y_grid)'-...
            dotX_grid+lambda_delta*X_grid,L*C*X_grid,zeros(nx,1);...
            (L*C*X_grid)',-lambda_delta_epsilon*X_grid,zeros(nx,1);...
            zeros(1,nx),zeros(1,nx),lambda_delta_epsilon*eps_sq-lambda_delta*delta_sq];
    % ineq = H_bar+...
    %        [(A_grid*X_grid+B_grid*Y_grid)+(A_grid*X_grid+B_grid*Y_grid)'-...
    %         dotX_grid+lambda_delta*X_grid,L*X_grid;...
    %         (L*X_grid)',-lambda_delta_epsilon*X_grid];

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_delta_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_delta_RPI_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_delta_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_delta_RPI_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_delta_RPI_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
