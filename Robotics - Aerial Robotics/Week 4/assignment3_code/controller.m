function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

Kd = 40;  % Derivative gains
Kp = 400;  % Propotional gains
Kd_attitude = 2;
Kp_attitude = 100;

Ep = des_state.pos - state.pos;  % Position error
Ev = des_state.vel - state.vel;  % Velocity error

r_des_ddot = des_state.acc + Kd*Ev + Kp*Ep;  % Eqn. (17)

psi_t = des_state.yaw;
phi_des =  (r_des_ddot(1)*psi_t - r_des_ddot(2))/params.gravity;  %Eqn. (14a)
theta_des = (r_des_ddot(1) + r_des_ddot(2)*psi_t)/params.gravity;  % Eqn. (14b)

e_rot = [phi_des; theta_des; des_state.yaw] - state.rot;  % Rot error
e_omega = [0; 0; des_state.yawdot] - state.omega;  % Omega error

F = params.mass*(params.gravity + r_des_ddot(3));  % Eqn.(13)

M = Kp_attitude*e_rot + Kd_attitude*e_omega;  % Eqn. (10)


% =================== Your code ends here ===================

end
