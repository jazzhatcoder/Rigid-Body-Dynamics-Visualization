% function dcm = quat2dcm(q)
%     % Convert quaternion to direction cosine matrix
%     % Quaternion format is [q0, q1, q2, q3] where q0 is scalar
% 
%     q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
% 
%     dcm = zeros(3,3);
% 
%     dcm(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
%     dcm(1,2) = 2*(q1*q2 - q0*q3);
%     dcm(1,3) = 2*(q1*q3 + q0*q2);
% 
%     dcm(2,1) = 2*(q1*q2 + q0*q3);
%     dcm(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
%     dcm(2,3) = 2*(q2*q3 - q0*q1);
% 
%     dcm(3,1) = 2*(q1*q3 - q0*q2);
%     dcm(3,2) = 2*(q2*q3 + q0*q1);
%     dcm(3,3) = q0^2 - q1^2 - q2^2 + q3^2;
% end

function dcm = quat2dcm(q)
    % Convert quaternion to direction cosine matrix
    % Quaternion format is [q0, q1, q2, q3] where q0 is scalar
    
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    dcm = zeros(3,3);
    
    dcm(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
    dcm(1,2) = 2*(q1*q2 - q0*q3);
    dcm(1,3) = 2*(q1*q3 + q0*q2);
    
    dcm(2,1) = 2*(q1*q2 + q0*q3);
    dcm(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
    dcm(2,3) = 2*(q2*q3 - q0*q1);
    
    dcm(3,1) = 2*(q1*q3 - q0*q2);
    dcm(3,2) = 2*(q2*q3 + q0*q1);
    dcm(3,3) = q0^2 - q1^2 - q2^2 + q3^2;
end