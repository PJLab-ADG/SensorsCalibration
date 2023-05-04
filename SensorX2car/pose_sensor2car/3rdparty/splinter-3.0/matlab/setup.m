% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

function setup()
    p = mfilename('fullpath');
    [upper_path, ~, ~] = fileparts(p);
    [upper_path, ~, ~] = fileparts(upper_path);
    splinter_path = upper_path;
    
    % development / debug use
    %splinter_path = '/home/anders/SPLINTER/build/debug/splinter-matlab';
    
    % Read version file. Name of library file depends on the version.
    version_file = fullfile(splinter_path, 'version');
    version_file_id = fopen(version_file, 'r');
    version = fscanf(version_file_id, '%d-%d');
    fclose(version_file_id);
    
    major_version = version(1);
    minor_version = version(2);

    % Add the directory containing the MATLAB interface of SPLINTER to the
    % search path that MATLAB searches through to find .m files.
    addpath(fullfile(splinter_path, 'matlab'));

    windows = ispc();
    mac = ismac();
    linux = isunix() && ~mac;

    % Detect architecture. Linux and MAC does not have x86 builds.
    arch = 'x86-64';
    if(strcmp('PCWIN', computer()))
        arch = 'x86';
    end

    % Header file is at the same location no matter the OS
    header_file = fullfile(splinter_path, 'include', 'cinterface.h');

    lib_base_name = strcat('splinter-', int2str(major_version));
    lib_base_name = strcat(lib_base_name, '-');
    lib_base_name = strcat(lib_base_name, int2str(minor_version));
    
    if(windows)
        lib_file_dir = fullfile(splinter_path, 'lib', 'windows', arch);
        lib_file = fullfile(lib_file_dir, strcat(lib_base_name, '.dll'));
    elseif(linux)
        lib_file_dir = fullfile(splinter_path, 'lib', 'linux', arch);
        lib_file = fullfile(lib_file_dir, strcat('lib', strcat(lib_base_name), '.so'));
    elseif(mac)
        lib_file_dir = fullfile(splinter_path, 'lib', 'osx', arch);
        lib_file = fullfile(lib_file_dir, strcat('lib', strcat(lib_base_name), '.so'));
    else
        lib_file_dir = fullfile(splinter_path, 'lib', 'linux', arch);
        lib_file = fullfile(lib_file_dir, strcat('lib', strcat(lib_base_name), '.so'));
    end

    % The Splinter class is implemented as a Singleton
    Splinter.get_instance().load(lib_file, header_file);
end
