classdef DistanceVisualizer < matlab.System
    % A System object to visualize an 8x8 distance matrix as a grayscale image
    % with inverted colors and a dynamic maximum value for normalization.
    
    %#codegen
    properties (Access = private)
        figHandle       % Handle to the figure window
        imgHandle       % Handle to the image object
        colorbarHandle  % Handle to the colorbar
        maxDistance = 2; % Initial maximum distance (meters)
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Create the figure for visualization
            if coder.target('MATLAB')
                obj.figHandle = figure('Name', 'Distance Visualizer', ...
                                       'NumberTitle', 'off', ...
                                       'Visible', 'on', ...
                                       'Position', [100, 100, 450, 400]);

                % Create an axes object for the image
                ax = axes('Parent', obj.figHandle, ...
                          'XTick', [], 'YTick', []);
                obj.imgHandle = imagesc(zeros(8, 8), 'Parent', ax);
                colormap(ax, 'gray'); % Use grayscale colormap
                caxis(ax, [0, 1]);    % Fix color axis range for consistent scaling
                axis(ax, 'image');    % Equal aspect ratio

                % Add a colorbar for visualization
                obj.colorbarHandle = colorbar(ax);
                ylabel(obj.colorbarHandle, 'Distance (m)');
                obj.updateColorbar(); % Set initial colorbar ticks
            end
        end

        function stepImpl(obj, distanceMatrix)
            % Update the image with the new distance array

            % Validate the input size
            if ~isequal(size(distanceMatrix), [8, 8])
                error('Input must be an 8x8 matrix.');
            end

            % Update the maximum distance if a value greater than the current max appears
            maxValueInData = max(distanceMatrix(:));
            if maxValueInData > obj.maxDistance
                obj.maxDistance = maxValueInData;
                obj.updateColorbar(); % Update colorbar ticks when maxDistance changes
            end

            % Normalize the distance array using the dynamic maxDistance
            normalizedArray = min(distanceMatrix, obj.maxDistance) / obj.maxDistance;

            % Invert the normalized values for grayscale display
            invertedArray = 1 - normalizedArray;

            % Update the image data
            if coder.target('MATLAB') && isvalid(obj.figHandle)
                set(obj.imgHandle, 'CData', invertedArray);
                caxis([0, 1]); % Keep color axis consistent
                drawnow limitrate; % Efficiently update the plot
            end
        end

        function releaseImpl(obj)
            % Close the figure window
            if coder.target('MATLAB') && isvalid(obj.figHandle)
                close(obj.figHandle);
            end
        end

        function validateInputsImpl(~, distanceMatrix)
            % Validate the input as an 8x8 matrix
            validateattributes(distanceMatrix, {'double', 'single'}, ...
                {'real', 'size', [8, 8]}, '', 'distanceMatrix');
        end
    end

    methods (Access = private)
        function updateColorbar(obj)
            % Update the colorbar to display actual distance values in meters
            if coder.target('MATLAB') && isvalid(obj.colorbarHandle)
                % Set colorbar ticks corresponding to actual distances
                obj.colorbarHandle.Ticks = linspace(0, 1, 5); % Normalized scale
                obj.colorbarHandle.TickLabels = arrayfun(@(x) sprintf('%.2f', x), ...
                    linspace(0, obj.maxDistance, 5), 'UniformOutput', false);
                
                % Update colorbar label with the current max distance
                ylabel(obj.colorbarHandle, sprintf('Distance (m), Max: %.2f', obj.maxDistance));
            end
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
