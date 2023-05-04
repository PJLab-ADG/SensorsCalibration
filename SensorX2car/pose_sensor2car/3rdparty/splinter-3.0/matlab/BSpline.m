% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

classdef BSpline
    properties (Access = protected)
        Handle
        
        Load_init_function = 'splinter_bspline_load_init';
        Eval_col_major_function = 'splinter_bspline_eval_col_major';
        Eval_jacobian_col_major_function = 'splinter_bspline_eval_jacobian_col_major';
        Eval_hessian_col_major_function = 'splinter_bspline_eval_hessian_col_major';
        Get_num_variables_function = 'splinter_bspline_get_num_variables';
        Save_function = 'splinter_bspline_save';
        Delete_function = 'splinter_bspline_delete';
    end

    methods
        % Constructor. Creates an instance of the BSpline class in the
        % library, either loading from file if handle_or_filename is a
        % file, or with a handle (used internally only).
        function obj = BSpline(handle_or_filename)
            
            if(ischar(handle_or_filename))
                filename = handle_or_filename;
                obj.Handle = Splinter.get_instance().call(obj.Load_init_function, filename);
            
            else
                handle = handle_or_filename;
                obj.Handle = handle;
            end
        end
        
        % Evaluate the B-spline at x
        % Supports batch evaluation:
        % x = [0 0; 1 1] will evaluate the approximant in both [0 0] and
        % [1 1], and return a nx1 matrix with the values, where n is the
        % number of rows in x (or points you evaluated it in).
        function r = eval(obj, x)
            lib_p = Utilities.matrix_to_c_array(x);
            num_points = numel(x) / obj.get_num_variables();
            temp = Splinter.get_instance().call(obj.Eval_col_major_function, obj.Handle, lib_p, numel(lib_p.value));
            r = Utilities.c_array_to_matrix(temp, num_points, 1);
        end
        
        % Evaluate the Jacobian at x
        function r = eval_jacobian(obj, x)
            lib_p = Utilities.matrix_to_c_array(x);
            num_points = numel(x) / obj.get_num_variables();
            temp = Splinter.get_instance().call(obj.Eval_jacobian_col_major_function, obj.Handle, lib_p, numel(lib_p.value));
            r = Utilities.c_array_to_matrix(temp, num_points, obj.get_num_variables());
        end
        
        % Evaluate the Hessian at x
        function r = eval_hessian(obj, x)
            lib_p = Utilities.matrix_to_c_array(x);
            num_points = numel(x) / obj.get_num_variables();
            temp = Splinter.get_instance().call(obj.Eval_hessian_col_major_function, obj.Handle, lib_p, numel(lib_p.value));
            r = Utilities.c_array_to_3d_matrix(temp, obj.get_num_variables(), obj.get_num_variables(), num_points);
        end

        function r = get_num_variables(obj)
            r = Splinter.get_instance().call(obj.Get_num_variables_function, obj.Handle);
        end
        
        function save(obj, fileName)
            Splinter.get_instance().call(obj.Save_function, obj.Handle, fileName);
        end

        % Destructor. Deletes the internal BSpline object.
        function delete(obj)
            if(obj.Handle ~= -1)
                Splinter.get_instance().call(obj.Delete_function, obj.Handle);
            end
        end
    end
end