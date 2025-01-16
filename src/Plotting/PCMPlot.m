classdef PCMPlot < matlab.System
    %Visualize audio waveform from PCM data

    %   Copyright 2024 The MathWorks, Inc.

    %#codegen
    properties (Access = protected)
        visObj  % Handle to the figure or plot
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Setup the visualization
            if coder.target('MATLAB')
                % Create the figure and axes for plotting
                obj.visObj = figure('Name', 'PCM Audio Waveform', ...
                                    'NumberTitle', 'off', ...
                                    'Visible', 'on', ...
                                    'Position', [100, 100, 800, 400]);
                ax = axes(obj.visObj, 'NextPlot', 'replacechildren');
                plot(ax, NaN, 'b', 'LineWidth', 1.5); % Initialize empty plot
                xlabel(ax, 'Samples');
                ylabel(ax, 'Amplitude');
                title(ax, 'Audio PCM Waveform');
                grid(ax, 'on');
            end
        end

        function stepImpl(obj, pcmData)
            % Update the plot with PCM data
            if coder.target('MATLAB')
                if isvalid(obj.visObj)
                    ax = obj.visObj.CurrentAxes; % Access current axes
                    plot(ax, 1:length(pcmData), pcmData, 'b', 'LineWidth', 1.5);
                    drawnow limitrate; % Update the plot efficiently
                end
            end
        end

        function releaseImpl(obj)
            % Clean up resources
            if coder.target('MATLAB')
                if isvalid(obj.visObj)
                    close(obj.visObj);
                end
                obj.visObj = [];
            end
        end

        %% Validate input and properties
        function validateInputsImpl(~, pcmData)
            % Validate the PCM input as a real vector
            validateattributes(pcmData, {'double', 'single'}, ...
                {'real', 'vector', 'nonempty'}, '', 'PCM data', 1);
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
