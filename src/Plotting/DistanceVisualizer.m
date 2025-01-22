classdef DistanceVisualizer < matlab.System
    % A System object to visualize 8x8 distance matrix as a grayscale image
    % with inverted colors.

    %   Copyright 2024 The MathWorks, Inc.

    %#codegen
    properties (Access = private)
        figHandle   % Handle to the figure window
        imgHandle   % Handle to the image object
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Create the figure for visualization
            if coder.target('MATLAB')
                obj.figHandle = figure('Name', 'Distance Visualizer', ...
                                        'NumberTitle', 'off', ...
                                        'Visible', 'on', ...
                                        'Position', [100, 100, 400, 400]);

                % Create an axes object for the image
                ax = axes('Parent', obj.figHandle, ...
                          'XTick', [], 'YTick', []);
                obj.imgHandle = imagesc(zeros(8, 8), 'Parent', ax);
                colormap(ax, 'gray'); % Use grayscale colormap
                caxis(ax, [0, 1]);    % Fix color axis range for consistent scaling
                axis(ax, 'image');    % Equal aspect ratio
            end
        end

        function stepImpl(obj, distanceMatrix)
            % Update the image with the new distance array

            % Validate the input size
            if ~isequal(size(distanceMatrix), [8, 8])
                error('Input must be an 8x8 matrix.');
            end

            % Normalize the distance array to [0, 1]
            minValue = min(distanceMatrix(:));
            maxValue = max(distanceMatrix(:));
            normalizedArray = (distanceMatrix - minValue) / (maxValue - minValue);

            % Invert the normalized values
            invertedArray = 1 - normalizedArray;

            % Update the image data
            if coder.target('MATLAB') && isvalid(obj.figHandle)
                set(obj.imgHandle, 'CData', invertedArray);
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

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
