%% Tube evolution test
% Initialize matlab interface
clear;
close all;
clc;

% Define tube contraction and growth factors
rho_c = 1.3;
w_bar_c = 10;

% Define time to evaluate tube evolution
t = 10;

% Define sampling time to evaluate tube in discrete time
ts = 0.01;


%% Continuous-time tube evolution
tube_ct = tube_evolution_ct(rho_c,w_bar_c,t);
fprintf("CT tube at %f: %f\n", t, tube_ct);


%% Discrete-time tube evolution
tube_dt = tube_evolution_dt(rho_c,w_bar_c,ts,t);
fprintf("DT tube at %f: %f\n", t, tube_dt);


%% Functions
function tube = tube_evolution_ct(rho_c,w_bar_c,t)
  tube = w_bar_c/rho_c*(1-exp(-rho_c*t));
end

function tube = tube_evolution_dt(rho_c,w_bar_c,ts,t)
  rho_d = exp(-rho_c*ts);
  w_bar_d = w_bar_c/rho_c*(1-rho_d);
  tube = 0;
  for i=1:t/ts
    tube = rho_d*tube+w_bar_d;
  end
end
