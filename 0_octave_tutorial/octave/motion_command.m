function [x] = motion_command(x, u)
% Updates the robot pose according to the motion model
% x: 3x1 vector representing the robot pose [x; y; theta]
% u: struct containing odometry reading (r1, t, r2).
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

%TODO: update x according to the motion represented by u

%TODO: remember to normalize theta by calling normalize_angle for x(3)

x_old=x(1)
y_old=x(2)
yaw_old=x(3)

x_new = x_old + u.t * cos(yaw_old + u.r1)
y_new = y_old + u.t * sin(yaw_old + u.r1)
yaw_new = yaw_old + u.r1 + u.r2

x(1)=x_new
x(2)=y_new
x(3)=yaw_new

end
