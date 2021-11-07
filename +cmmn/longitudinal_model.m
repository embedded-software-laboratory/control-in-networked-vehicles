function [ model ] = longitudinal_model(Ts)
%LONGITUDINAL_MODEL Returns state space model for one vehicle

A = zeros(3,3);
B = zeros(3,1);
C = eye(2,3);
D = 0;
vehicle_model_ss = ss(A, B, C, D);

% Convert to discrete time
model = c2d(vehicle_model_ss,Ts);
% Absorb delay into system dynamics
model = absorbDelay(model);
model.InputName = 'VelocityIn';
model.InputUnit = 'm/s';
model.OutputName{1} = 'Distance';
model.OutputUnit{1} = 'm';
model.OutputName{2} = 'Velocity';
model.OutputUnit{2} = 'm/s';
end