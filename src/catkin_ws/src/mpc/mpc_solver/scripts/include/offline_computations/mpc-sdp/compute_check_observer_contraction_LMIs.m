function result = compute_check_observer_contraction_LMIs(do_print,do_check,model,rho_o,P_delta_xu,Agrid,L,n_uxABXYgrid,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = n_uxABXYgrid;

  % Get model variables
  C = model.get_C();

  for i=1:n_lmi
    A_grid = Agrid(:, :, i);
    ineq = (A_grid-L*C)'*P_delta_xu+P_delta_xu*(A_grid-L*C)+2*rho_o*P_delta_xu;

    if ~do_check
      result = [result;ineq<=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_observer_contraction_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      max_eig(i) = max(eig(ineq));
      if (max_eig(i) > tol)
        result = [result;[i,max_eig(i)]];
        if do_print
          fprintf("[compute_check_observer_contraction_LMIs] Maximum eigenvalue of LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_observer_contraction_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_observer_contraction_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_observer_contraction_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
