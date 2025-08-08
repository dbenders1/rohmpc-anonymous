function print_diagnostics(diagnostics)
  fprintf(" => ");
  if diagnostics.problem == 0
    cprintf([0,1,0], "optimization successful!\n");
  else
    err_msg = yalmiperror(diagnostics.problem);
    cprintf([1,0,0], "optimization unsuccessful (yalmip error code %i: %s)\n",diagnostics.problem,[lower(err_msg(1)),err_msg(2:end-1)]);
  end
end
