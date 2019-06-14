function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);

for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  t   = u.t + normrnd(0, r1Noise);
  r1  = u.r1 + normrnd(0, r1Noise);
  r2  = u.r2 + normrnd(0, r2Noise);
  particles(i).pose += [t * cos( normalize_angle( particles(i).pose(3) ) + r1 );
                        t * sin( normalize_angle( particles(i).pose(3) ) + r1 );
                        normalize_angle( r1 + r2)];

end

end
