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