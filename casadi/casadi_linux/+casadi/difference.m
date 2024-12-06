function varargout = difference(varargin)
    %DIFFERENCE \\bried Return all elements of a that do not occur in b, preserving 
    %
    %  {MX} = DIFFERENCE({MX} a, {MX} b)
    %
    %order
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/mx.hpp#L849
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/mx.hpp#L849-L851
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(904, varargin{:});
end
