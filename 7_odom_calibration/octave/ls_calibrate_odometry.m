% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 

  % TODO: initialize H and b of the linear system
  b = 0;
  H = 0;
  
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  N = size(Z)(1);
  for i=1:N
    err = error_function(i, X, Z);
    jac = jacobian(i, Z);
    b += err'*eye(3)*jac;
    H += jac'*eye(3)*jac;
  endfor
  % TODO: solve and update the solution
  X += reshape( -inv(H)*b' , 3 , 3)';          % WHY TRANSPOSE WAS NEEDED? I COULDN'T FIND IT IN THE COURSE?
  end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  u_truth = Z(i,1:3)';
  u_odom = Z(i,4:6)';
  e = u_truth - X*u_odom;
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  J = zeros(3,9);
  J(1,1:3) = Z(i,4:6); % = u_odom
  J(2,4:6) = Z(i,4:6); % = u_odom
  J(3,7:9) = Z(i,4:6); % = u_odom
  J = -J;
end
