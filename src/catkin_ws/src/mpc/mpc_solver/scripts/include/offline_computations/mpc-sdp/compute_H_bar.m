function [H_bar,H_top] = compute_H_bar(do_print,model,L,h,tol)
  n_bytes = 0;

  % Construct H_bar
  nx = model.nx;
  neta = model.neta;
  F = model.get_F();
  [eta_min, eta_max] = model.get_etaminmax();
  eta_min_max = [eta_min,eta_max];
  eta_abs_max = max(abs(eta_min_max),[],2);
  % L_diag = diag(L);
  % h = sqrt(sum((L_diag.*F*eta_abs_max).^2)*neta);
  % h = 0.05;
  H_diag = zeros(neta,1);
  for i=1:neta
    H_diag(i) = (L(i,i)*F(i,i)*eta_abs_max(i))^2*neta/h;
  end
  H_top = blkdiag(diag(H_diag),tol*eye(nx-neta),tol*eye(nx));
  H_bar = blkdiag(H_top,h);
  if do_print
      fprintf("[compute_H_bar] Constructed H_bar\n");
  end

  % Check validity of H_bar
  [etagrid, n_etagrid] = model.get_etagrid();
  n_lmi = n_etagrid;
  i = 0;
  for j=1:n_etagrid
    i = i + 1;
    eta = etagrid(:, j);
    ineq = [zeros(nx,nx),zeros(nx,nx),L*F*eta;
            zeros(nx,nx),zeros(nx,nx),zeros(nx,1);
            (L*F*eta)',zeros(1,nx),0] - H_bar;

    max_eig(i) = max(eig(ineq));
    if (max_eig(i) > tol)
      if do_print
        fprintf("[compute_H_bar] Maximum eigenvalue of H_bar-delta-RPI LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
      end
    end
    if do_print && rem(i,200) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_H_bar] Checked H_bar-delta-RPI LMI %i/%i\n",i,n_lmi);
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_H_bar] Checked H_bar-delta-RPI LMI %i/%i\n",i,n_lmi);
  end
end
