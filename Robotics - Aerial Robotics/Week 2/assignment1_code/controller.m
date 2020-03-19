function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE

% FOR HOVER
% u = params.mass*params.gravity;

% FOR STEP
e = s_des - s;
Kp = 110;
Kv = 15;
Zdes = 1;
u = params.mass*(params.gravity+(Kp*e(1))+(Kv*e(2))+ Zdes);


end

