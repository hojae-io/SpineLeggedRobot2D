function varargout = nlpsol_option_type(varargin)
    %NLPSOL_OPTION_TYPE [INTERNAL] 
    %
    %  char = NLPSOL_OPTION_TYPE(char name, char op)
    %
    %Get type info for a particular option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L824
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L824-L826
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(835, varargin{:});
end
