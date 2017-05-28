classdef Maccepavd1DOF
    %MACCEPAVD1DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rigidBodyTree;
        
    end
    
    methods
        function self = Maccepavd1DOF()
            self.rightBodyTree = robotics.RigidBodyTree;
            baselink = robotics.RigidBody('baselink');
            
        end
    end
    
end

