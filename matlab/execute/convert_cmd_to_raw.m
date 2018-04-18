function [ uraw ] = convert_cmd_to_raw( u )
%CONVERT_CMD_TO_RAW convert command to raw values
%   input:  u - commands, 3 by N_steps matrix
%   output: uraw - raw commands, 4 by N_steps matrix

    umin = [900;900;0;0];
    umax = [2100;2100;255;255];
    uraw = zeros(4, size(u,2));
    uraw(1,:) = round(u(1,:)*1800/pi + 1500);
    uraw(2,:) = round(u(2,:)*1800/pi + 900);
    for i = 1:size(u,2)
        if u(3,i) <= 0.5
            uraw(3,i) = round(u(3,i)*255/0.5) ;
            uraw(4,i) = 0;
        else
            uraw(3,i) = 255;
            uraw(4,i) = round((u(3,i)-0.5)*255/0.5);
        end
        uraw(:,i) = max( min( uraw(:,i), umax ), umin); %limit the commands for safety
    end
    
end

