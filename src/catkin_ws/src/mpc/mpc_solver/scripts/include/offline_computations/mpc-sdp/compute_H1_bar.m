function [H1_bar,H1_top] = compute_H1_bar(do_print,model,L,h1,tol)
  n_bytes = 0;

  % Construct H1_bar
  nx = model.nx;
  neta = model.neta;
  F = model.get_F();
  [eta_min, eta_max] = model.get_etaminmax();
  eta_min_max = [eta_min,eta_max];
  eta_abs_max = max(abs(eta_min_max),[],2);
  % L_diag = diag(L);
  % h1 = sqrt(sum((L_diag.*F*eta_abs_max).^2)*neta);
  % h1 = 0.05;
  H1_diag = zeros(neta,1);
  for i=1:neta
    H1_diag(i) = (L(i,i)*F(i,i)*eta_abs_max(i))^2*neta/h1;
  end
  H1_top = blkdiag(diag(H1_diag),tol*eye(nx-neta));
  H1_bar = blkdiag(H1_top,h1);
  if do_print
    fprintf("[compute_H1_bar] Constructed H1_bar\n");
  end

  % Check validity of H1_bar
  [etagrid, n_etagrid] = model.get_etagrid();
  n_lmi = n_etagrid;
  i = 0;
  for j=1:n_etagrid
    i = i + 1;
    eta = etagrid(:, j);
    ineq = [zeros(nx,nx),-L*F*eta;
            (-L*F*eta)',zeros(1,1)] - H1_bar;

    max_eig(i) = max(eig(ineq));
    if (max_eig(i) > tol)
      result = [result;[i,max_eig(i)]];
      if do_print
        fprintf("[compute_H1_bar] Maximum eigenvalue of H1_bar-epsilon-RPI LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
      end
    end
    if do_print && rem(i,200) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_H1_bar] Checked H1_bar-epsilon-RPI LMI %i/%i\n",i,n_lmi);
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_H1_bar] Checked H1_bar-epsilon-RPI LMI %i/%i\n",i,n_lmi);
  end
end
