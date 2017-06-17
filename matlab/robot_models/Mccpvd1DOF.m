classdef Mccpvd1DOF < arm1
    %MACCEPAVD1DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %rigidBodyTree;
        
        %link_len = 0.15;
        %mass = 1;
        %com = 0.1;
    end
    
    methods
        function self = Mccpvd1DOF(param)
            if( isfield(param,'link_len') ), param_arm.link_len = param.link_len;
            else, param_arm.link_len = 0.15;
            end
            if( isfield(param,'mass') ), param_arm.mass = param.mass;
            else, param_arm.mass = 1;
            end
            if( isfield(param,'com') ), param_arm.com = param.com;
            else, param_arm.com = 0.1;
            end
            
            self = self@arm1(param_arm);
            self.rigidBodyTree = self.init_rigidBodyTree();
        end
        
    end
    
end

