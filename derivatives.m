% change scalar to vector equation
% function dstate = derivatives(state, I) %variable=function name(parameters)
%     % Extract state variables
%     q = state(1:4);  % quaternion (scalar first)
%     w = state(5:7);  % angular velocity
% 
%     % Quaternion kinematics: dq/dt = 0.5 * q ⊗ w_q
%     % w_q is pure quaternion [0; w']
%     w_q = [0; w'];
%     q_matrix = [q(1), -q(2), -q(3), -q(4);
%                 q(2),  q(1), -q(4),  q(3);
%                 q(3),  q(4),  q(1), -q(2);
%                 q(4), -q(3),  q(2),  q(1)];
%     dq = 0.5 * q_matrix * w_q;
% 
%     % Euler's equations: I*dw/dt + w × (I*w) = 0
%     % For principal axes, this simplifies to:
%     dw = zeros(3,1);
%     dw(1) = (I(2) - I(3))*w(2)*w(3)/I(1);
%     dw(2) = (I(3) - I(1))*w(3)*w(1)/I(2);
%     dw(3) = (I(1) - I(2))*w(1)*w(2)/I(3);
% 
% 
%     dstate = [dq', dw'];
% end

function dstate = derivatives(state, I_tensor)
    % Extract state variables
    q = state(1:4);  % quaternion (scalar first)
    w = state(5:7);  % angular velocity
    
    % Quaternion kinematics: dq/dt = 0.5 * q ⊗ w_q
    % w_q is pure quaternion [0; w']
    w_q = [0; w'];
    q_matrix = [q(1), -q(2), -q(3), -q(4);
                q(2),  q(1), -q(4),  q(3);
                q(3),  q(4),  q(1), -q(2);
                q(4), -q(3),  q(2),  q(1)];
    dq = 0.5 * q_matrix * w_q;
    
    % Euler's equations for general case with full inertia tensor:
    % I*dw/dt + w × (I*w) = 0
    Iw = I_tensor * w(:);  % Ensure w is a column vector
    w_cross_Iw = cross(w, Iw');  % Transpose Iw back to row format for cross product
    
    % Solve for dw: dw = inv(I) * (-w × (I*w))
    dw = -I_tensor \ w_cross_Iw(:);  % Convert to column vector
    
    dstate = [dq', dw'];
end