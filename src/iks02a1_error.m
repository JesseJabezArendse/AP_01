classdef iks02a1_error < matlab.System
    methods (Access = protected)
        function stepImpl(obj, trigger)
            if trigger == true
                % Display the custom message
                disp("IKS02A1 Error - Unplug and Replug the Nucleo!");
                
                % Stop the simulation
                error('Simulink:NucleoError', 'Unplug and Replug the Nucleo!');
            end
        end
    end
end
