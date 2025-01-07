function auto_program(filename)
    if ispc
        auto_program_win(filename)
    end
    if isunix
        auto_program_linux(filename)
    end
    if ismac
        auto_program_mac(filename)
    end
end