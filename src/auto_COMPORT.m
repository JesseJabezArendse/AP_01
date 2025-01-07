function STLINK_COMPORT = auto_COMPORT()
    if ispc
        STLINK_COMPORT = auto_COMPORT_win();
    end
    if isunix
        STLINK_COMPORT = auto_COMPORT_linux();
    end
    if ismac
        STLINK_COMPORT = auto_COMPORT_mac();
    end
end