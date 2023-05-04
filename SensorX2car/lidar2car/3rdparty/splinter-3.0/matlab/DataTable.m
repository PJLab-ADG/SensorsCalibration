% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

classdef DataTable < handle
    properties (Access = private)
        Handle
        
        X_dim
        
        % Samples store the samples that are awaiting transfer to the back
        % end, Num_samples is tracking how many samples we are storing
        Samples
        Num_samples
        
        Init_function = 'splinter_datatable_init';
        Get_num_variables_function = 'splinter_datatable_get_num_variables';
        Get_num_samples_function = 'splinter_datatable_get_num_samples';
        Add_samples_col_major_function = 'splinter_datatable_add_samples_col_major';
        Delete_function = 'splinter_datatable_delete';
    end
    
    methods
        % Constructor. Creates an instance of the DataTable class in the
        % library.
        function obj = DataTable(x, y)
            obj.Handle = Splinter.get_instance().call(obj.Init_function);

            if ~ismatrix(x) || ~ismatrix(y)
                error('x and y must be matrices!');
            end

            x_size = size(x);
            y_size = size(y);
            if x_size(1) ~= y_size(1)
                error('x and y must have equal number of rows!');
            end

            obj.Samples = [x y];
            obj.X_dim = x_size(2);
            obj.Num_samples = x_size(1);
            
            sample_ptr = libpointer('doublePtr', obj.Samples);
            % We need to pass the number of rows in the matrix to the
            % library because of the way MatLab stores matrices in
            % memory. It stores them column-major, which means that we
            % need to know the number of rows to reconstruct the data
            % in the back end.
            Splinter.get_instance().call(obj.Add_samples_col_major_function, obj.Handle, sample_ptr, obj.Num_samples, obj.X_dim);
        end
        
        function r = get_num_variables(obj)
            r = Splinter.get_instance().call(obj.Get_num_variables_function, obj.Handle); 
        end
        
        function r = get_num_samples(obj)
            r = Splinter.get_instance().call(obj.Get_num_samples_function, obj.Handle); 
        end
        
        function r = get_handle(obj)
            r = obj.Handle;
        end
        
        % Destructor. Deletes the internal DataTable object.
        function delete(obj)
            if(obj.Handle ~= -1)
                Splinter.get_instance().call(obj.Delete_function, obj.Handle);
            end
        end
    end
end