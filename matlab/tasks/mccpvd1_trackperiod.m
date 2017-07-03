classdef mccpvd1_trackperiod
    %MCCPVD1_TRACKPERIOD tracking periodic movements
    
    properties
        
    end
    
    methods
        function obj = mccpvd1_trackperiod()
            
        end
    end
    
    methods (Static)
        function [x, xdot] = generate_sin(amp, freq, t)
            x = amp*sin(freq.*t);
            xdot = amp*freq*cos(freq.*t);
        end
        
        function error = track_error(xref, x, n)
            error = norm(xref(:,n)-x,2);
        end
    end
    
end

