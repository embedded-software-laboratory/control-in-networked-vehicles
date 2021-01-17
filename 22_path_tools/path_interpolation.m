%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function interp = path_interpolation(s_queried, start_point, end_point)
    
    s_start = start_point.s;
    s_end = end_point.s;

    delta_s = s_end - s_start;
    tau = (s_queried - s_start) / delta_s;
    
    tau2 = tau * tau;
    tau3 = tau * tau2;
    
    position_start_x = start_point.pose.x;
    position_start_y = start_point.pose.y;
    position_end_x = end_point.pose.x;
    position_end_y = end_point.pose.y;
    
    velocity_start_x = cos(start_point.pose.yaw) * delta_s;
    velocity_start_y = sin(start_point.pose.yaw) * delta_s;
    velocity_end_x = cos(end_point.pose.yaw) * delta_s;
    velocity_end_y = sin(end_point.pose.yaw) * delta_s;
    
    
    % Hermite spline coefficients
    p0 = 2*tau3 - 3*tau2 + 1;
    m0 = tau3 - 2*tau2 + tau;
    p1 = -2*tau3 + 3*tau2;
    m1 = tau3 - tau2;
    
    % Hermite spline derivative coefficients
    dp0 = 6*tau2 - 6*tau;
    dm0 = 3*tau2 - 4*tau + 1;
    dp1 = -6*tau2 + 6*tau;
    dm1 = 3*tau2 - 2*tau;
    
    % Hermite spline second derivative coefficients
    ddp0 = 12*tau - 6;
    ddm0 = 6*tau - 4;
    ddp1 = -12*tau + 6;
    ddm1 = 6*tau - 2;    
    
    position_x     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    position_y     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
    velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_s;
    velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_s;
    acceleration_x = (position_start_x * ddp0 + velocity_start_x * ddm0 + position_end_x * ddp1 + velocity_end_x * ddm1) / (delta_s*delta_s);
    acceleration_y = (position_start_y * ddp0 + velocity_start_y * ddm0 + position_end_y * ddp1 + velocity_end_y * ddm1) / (delta_s*delta_s);
    
    yaw = atan2(velocity_y, velocity_x);
    speed = sqrt(velocity_x*velocity_x + velocity_y*velocity_y);
    curvature = (velocity_x * acceleration_y - velocity_y * acceleration_x) / (speed*speed*speed);
    
    interp.position_x = position_x;
    interp.position_y = position_y;
    interp.velocity_x = velocity_x;
    interp.velocity_y = velocity_y;
    interp.acceleration_x = acceleration_x;
    interp.acceleration_y = acceleration_y;
    interp.yaw = yaw;
    interp.speed = speed;
    interp.curvature = curvature;
end
