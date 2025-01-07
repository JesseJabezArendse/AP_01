function auto_program_mac(filename)
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