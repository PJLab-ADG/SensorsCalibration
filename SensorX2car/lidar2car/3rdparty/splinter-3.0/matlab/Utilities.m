% This file is part of the SPLINTER library.
% Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
%
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.

classdef Utilities
    methods(Static)
        % Returns a (column-major) (double) C array with the same contents as mat
        % Note: The returned c array is actually using the same underlying
        % data as MatLab, so be careful not to modify it in C (unless you
        % know what you're doing, in which case you wouldn't have bothered
        % about this warning anyway).
        function r = matrix_to_c_array(mat)
            lib_p = libpointer('doublePtr', mat);
            reshape(lib_p, 1, numel(mat));
            r = lib_p;
        end
        
        function r = matrix_to_c_uint_array(mat)
            lib_p = libpointer('uint32Ptr', mat);
            reshape(lib_p, 1, numel(mat));
            r = lib_p;
        end
        
        % Returns a MatLab matrix of size num_rows x num_cols with the data
        % from the C array of doubles c_array.
        % The underlying data is probably the same as is the case with
        % matrixToCArray, but we have not researched it, so don't take our
        % word for it.
        function r = c_array_to_matrix(c_array, num_rows, num_cols)
            reshape(c_array, num_rows, num_cols);
            r = c_array.value;
        end
        
        % Returns a MatLab matrix of size num_rows x num_cols x num_pages
        % with the data from the C array of doubles c_array.
        % The underlying data is probably the same as is the case with
        % matrixToCArray, but we have not researched it, so don't take our
        % word for it.
        function r = c_array_to_3d_matrix(c_array, num_rows, num_cols, num_pages)
            reshape(c_array, num_rows, num_cols*num_pages);
            r = reshape(c_array.value, num_rows, num_cols, num_pages);
        end
    end
end