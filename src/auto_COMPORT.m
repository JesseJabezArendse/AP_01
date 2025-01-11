function STLINK_COMPORT = auto_COMPORT()
    persistent savedCOMPort;
    dialogPrms = get_param(gcs+"/Communicating with STM32 Nucleo", 'DialogParameters');
    dialogPrmNames = fieldnames(dialogPrms);
    automate = get_param(gcs+"/Communicating with STM32 Nucleo", dialogPrmNames{1});
    automate = eval(automate);

    if automate
        if ispc
            STLINK_COMPORT = auto_COMPORT_win();
        end
        if isunix
            STLINK_COMPORT = auto_COMPORT_linux();
        end
        if ismac
            STLINK_COMPORT = auto_COMPORT_mac();
        end
    elseif isempty(savedCOMPort)
        options = serialportlist;
        % If no matching device was found, prompt the user to select a port
        [selection, ok] = listdlg(...
            'PromptString', 'Select the ST-Link COM Port:', ...
            'SelectionMode', 'single', ...
            'ListString', options, ...
            'Name', 'Select COM Port');
        
        % Handle user response
        if ok
            STLINK_COMPORT = options{selection};
            disp(['Selected ST-Link COM Port: ', STLINK_COMPORT]);
            savedCOMPort = STLINK_COMPORT;
        else
            error('No COM port selected. Aborting.');
        end
    else
        % Use the saved persistent COM port
        STLINK_COMPORT = savedCOMPort;
        disp(['ST-Link COM Port already set to: ', STLINK_COMPORT]);
    end
end