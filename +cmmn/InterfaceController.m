classdef (Abstract) InterfaceController < handle
% InterfaceController Interface for a controller class
    methods (Abstract) 
        [u,y] = step(obj,y_measured)
        setup(obj)
    end
end

