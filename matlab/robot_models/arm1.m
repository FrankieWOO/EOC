classdef arm1
    %ARM1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rigidBodyTree
        
        link_len
        mass
        com % center of mass
    end
    
    methods
        function self = arm1(param)
            if( isfield(param,'link_len') ), self.link_len = param.link_len; end
            if( isfield(param,'mass') ), self.mass = param.mass; end
            if( isfield(param,'com') ), self.com = param.com; end
            
            
        end
        
        function robot_tree = init_rigidBodyTree(self)
            robot_tree = robotics.RigidBodyTree;
            link1 = robotics.RigidBody('link1');
            link1.Mass = self.mass(1);
            link1.CenterOfMass = [self.com(1) 0 0];
            joint1 = robotics.Joint('joint1', 'revolute');
            setFixedTransform(joint1,[0 0 0 0] , 'dh');
            link1.Joint = joint1;
            basename = robot_tree.BaseName;
            addBody(robot_tree, link1, basename);
                        
            tool = robotics.RigidBody('tool');
            jnt_tool = robotics.Joint('jnt_tool','fixed');
            setFixedTransform(jnt_tool, trvec2tform([self.link_len(1), 0, 0]));
            tool.Joint = jnt_tool;
            addBody(robot_tree, tool, 'link1');
            
            robot_tree.Gravity = [0 0 -9.81];
        end
    end
    
end

