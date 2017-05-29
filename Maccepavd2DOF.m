classdef Maccepavd2DOF
    %MACCEPAVD1DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rigidBodyTree;
        
    end
    
    methods
        function self = Maccepavd2DOF()
            self.init_rigidBodyTree();
        end
        
        function init_rigidBodyTree(self)
            self.rigidBodyTree = robotics.RigidBodyTree;
            link1 = robotics.RigidBody('link1');
            link1.Mass = 1;
            link1.CenterOfMass = [0.1 0 0];
            joint1 = robotics.Joint('joint1', 'revolute');
            setFixedTransform(joint1,[0 0.15 0 0] , 'dh');
            link1.Joint = joint1;
            basename = self.rigidBodyTree.BaseName;
            addBody(self.rigidBodyTree, link1, basename);
            
        end
    end
    
end

