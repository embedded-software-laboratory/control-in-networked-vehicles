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
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

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
    
    interp.position_x =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    interp.position_y =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
end