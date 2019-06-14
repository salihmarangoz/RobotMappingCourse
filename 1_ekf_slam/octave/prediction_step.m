function [mu, sigma] = prediction_step(mu, sigma, u, first)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

N = (length(mu)-3)/2;
Fx = [ eye(3) zeros(3, N*2) ];

new_mu = mu + Fx' * [ u.t*cos( normalize_angle( mu(3) + u.r1 ) );
                      u.t*sin( normalize_angle( mu(3) + u.r1 ) );
                      normalize_angle( u.r1 + u.r2 ) ];

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = [0, 0, -u.t*sin( mu(3) + u.r1 );
      0, 0, u.t*sin( mu(3) + u.r1 );
      0, 0, 0 ];

% TODO: Construct the full Jacobian G
G = eye(2*N+3) + (Fx' * Gx * Fx);

% Motion noise
motionNoise = 0.25;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
% NOTE: If motion noise is very high, result of movement-sensor-movement... would be worse. So I changed the order to sensor-movement-sensor...
% My solution:
if (first == false)
  new_sigma = G*sigma*G' + R;
  sigma = new_sigma;
endif
% Original solution
%new_sigma = G*sigma*G' + R;
%sigma = new_sigma;

mu = new_mu;

end
