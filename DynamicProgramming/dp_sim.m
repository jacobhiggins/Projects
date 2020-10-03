DPinput.A = [1 0; 1 1];
DPinput.B = [0 1]';
DPinput.x_num_discrete = 10;
DPinput.u_num_discrete = 10;
DPinput.Tf_steps = 2;
DP_solver_matlab(DPinput);