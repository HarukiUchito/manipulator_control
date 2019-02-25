%EXAMPLE_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef Connection_interface < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = Connection_interface(varargin)
            this.objectHandle = Connection_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            Connection_mex('delete', this.objectHandle);
        end

        function varargout = open(this, varargin)
            [varargout{1:nargout}] = Connection_mex('open', this.objectHandle, varargin{:});
        end

        function varargout = receive(this, varargin)
            [varargout{1:nargout}] = Connection_mex('receive', this.objectHandle, varargin{:});
        end

        function varargout = send(this, varargin)
            [varargout{1:nargout}] = Connection_mex('send', this.objectHandle, varargin{:});
        end

        function varargout = out2DSpline(this, varargin)
            [varargout{1:nargout}] = Connection_mex('out2DSpline', this.objectHandle, varargin{:});
        end

        function varargout = outRangles(this, varargin)
            [varargout{1:nargout}] = Connection_mex('outRangles', this.objectHandle, varargin{:});
        end

        function varargout = calcTrajecotry(this, varargin)
            [varargout{1:nargout}] = Connection_mex('calcTrajecotry', this.objectHandle, varargin{:});
        end
        %{
        function varargout = getJacobian(this, varargin)
            [varargout{1:nargout}] = Connection_mex('getJacobian', this.objectHandle, varargin{:});
        end

        function varargout = inverseKinematics(this, varargin)
            [varargout{1:nargout}] = Connection_mex('inverseKinematics', this.objectHandle, varargin{:});
        end
        %}
    end
end