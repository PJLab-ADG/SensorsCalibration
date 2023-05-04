% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

classdef BSplineBuilder
    properties (Access = protected)
        Handle
        
        Init_function = 'splinter_bspline_builder_init';
        Set_degree_function = 'splinter_bspline_builder_set_degree';
        Set_smoothing_function = 'splinter_bspline_builder_set_smoothing';
        Set_alpha_function = 'splinter_bspline_builder_set_alpha';
        Set_knot_spacing_function = 'splinter_bspline_builder_set_knot_spacing';
        Build_function = 'splinter_bspline_builder_build';
        Delete_function = 'splinter_bspline_builder_delete';
    end

    methods
        % Constructor. Creates an instance of the BSpline class in the
        % library, using the samples in the data matrix.
        % degrees is the degrees of the b-spline (must be scalar or vector
        % of the same length as the dimensionality of the data)
        function obj = BSplineBuilder(x, y, degrees, smoothing, alpha, knot_spacing)
            datatable = DataTable(x, y);
            num_variables = datatable.get_num_variables();
            
            obj.Handle = Splinter.get_instance().call(obj.Init_function, datatable.get_handle());
            
            if(exist('degrees', 'var'))
                % Check if degrees is a scalar
                degrees_size = size(degrees);
                if (degrees_size(1) == 1 && degrees_size(2) == 1)
                    % Create a vector of size 1xnum_variables filled with
                    % the value in degrees
                    degrees(1:num_variables) = degrees;
                end
                Splinter.get_instance().call(obj.Set_degree_function, obj.Handle, Utilities.matrix_to_c_uint_array(degrees), num_variables);
            end
            
            if(exist('smoothing', 'var'))
                % These values are somewhat arbitrary, the important thing is
                % that the MatLab front end agrees with the C back end
                switch(lower(smoothing))
                    case 'none'
                        smoothing = 0;
                    case 'identity'
                        smoothing = 1;
                    case 'pspline'
                        smoothing = 2;
                end
                Splinter.get_instance().call(obj.Set_smoothing_function, obj.Handle, smoothing);
            end
            
            if(exist('alpha', 'var'))
                Splinter.get_instance().call(obj.Set_alpha_function, obj.Handle, alpha);
            end
            
            if(exist('knot_spacing', 'var'))
                % These values are somewhat arbitrary, the important thing is
                % that the MatLab front end agrees with the C back end
                switch(lower(knot_spacing))
                    case 'as_sampled'
                        knot_spacing = 0;
                    case 'equidistant'
                        knot_spacing = 1;
                    case 'experimental'
                        knot_spacing = 2;
                end
                Splinter.get_instance().call(obj.Set_knot_spacing_function, obj.Handle, knot_spacing);
            end
        end
        
        function r = build(obj)
            r = BSpline(Splinter.get_instance().call(obj.Build_function, obj.Handle));
        end

        % Destructor. Deletes the internal BSpline object.
        function delete(obj)
            if(obj.Handle ~= -1)
                Splinter.get_instance().call(obj.Delete_function, obj.Handle);
            end
        end
    end
end