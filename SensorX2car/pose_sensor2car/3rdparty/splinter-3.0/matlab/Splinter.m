% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

classdef Splinter < handle
    properties (Access = protected)
        Alias
        
        Get_error_function = 'splinter_get_error';
        Get_error_string_function = 'splinter_get_error_string';
    end

    methods(Access = private)
        function obj = Splinter
            obj.Alias = 'libSplinter';
        end 
    end
    
    methods(Static)
        function singleObj = get_instance
            persistent localObj
            if isempty(localObj) || ~isvalid(localObj)
                localObj = Splinter;
            end
            singleObj = localObj;
        end
    end
    
    methods
        function load(obj, lib_file, header_file, alias)
            if(obj.is_loaded())
                fprintf('Splinter is already loaded as %s\n', obj.Alias);
                fprintf('If you wish to change the alias you should unload and then load the library with the new alias.\n');
            else
                if(exist('alias', 'var'))
                   obj.Alias = alias;
                end
                
                fprintf('Loading Splinter as %s\n', obj.Alias);
                loadlibrary(lib_file, header_file, 'alias', obj.Alias)
            end
        end
        
        function r = call(obj, func, varargin)
            if(~obj.is_loaded())
                error('You need to load the library with Splinter.get_instance().load(libFile, headerFile [, alias]) before making calls to it!\n'); 
            end
            
            % strcmp == 1 if identical
            if(strcmp(func, ''))
                error('This function has not been implemented yet!'); 
            end
            
            %disp('Calling ');
            %disp(func);
            
            r = calllib(obj.Alias, func, varargin{:});
            
            % Check the internal error flag (set when trying to reference
            % an object that does not exist, which may happen when creating
            % an object, reloading the library, then trying to use the
            % object. The internal reference will then be gone, and the
            % reference invalid).
            if(calllib(obj.Alias, obj.Get_error_function))
                error_message = calllib(obj.Alias, obj.Get_error_string_function);
                error(error_message); 
            end
        end
        
        function unload(obj)
           unloadlibrary(obj.Alias);
        end
        
        function r = is_loaded(obj)
           r = libisloaded(obj.Alias);
        end
        
        function alias = get_alias(obj)
           alias = obj.Alias; 
        end
    end
end