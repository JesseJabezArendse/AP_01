
function auto_program_linux(filename)
    % Get connected drives and their details
    drives = get_linux_drives();

    % Iterate through the drives
    for i = 1:length(drives)
        drive_path = drives{i}.path;
        drive_name = drives{i}.name;

        % Check if the drive name ends with 'F411'
        if endsWith(drive_name, 'F411')
            disp("Found STM32 Drive: " + drive_name);

            % Copy the binary file to the drive
            copyfile(['binaries/' filename '.bin'], drive_path);

            pause(3); % Pause to ensure the file is copied
            disp("STM32 Programmed");
            return;
        end
    end

    % If no STM32 drive is found, throw an error
    error("No STM32 Nucleo F411 Found");
end

function drives = get_linux_drives()
    % Executes Linux command to list block devices with udev details
    [status, cmdout] = system('lsblk -o PATH,MODEL -n -p | grep -i "F411"');

    if status ~= 0
        error("Failed to fetch drive information");
    end

    % Parse the command output
    drives = {};
    lines = strsplit(cmdout, '\n');
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if isempty(line)
            continue;
        end

        % Split the line into PATH and MODEL
        parts = strsplit(line);
        drives{end+1} = struct('path', parts{1}, 'name', parts{2});
    end
end