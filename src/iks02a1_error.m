classdef iks02a1_error < matlab.System
    methods (Access = protected)
        function stepImpl(obj, u)
            if u == true
                % Display the custom message
                disp("IKS02A1 Error - Unplug and Replug the Nucleo!");
                
                % Stop the simulation
                error('Simulink:CustomError', 'Simulation stopped due to an IKS02A1 error.');
            end
            y = u; % Pass the input to the output if no error occurs
        end
    end
end
