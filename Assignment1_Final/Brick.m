% Author: Jackson Russelll 14250803
% Calss that aids with using the bricks as objects
classdef Brick
    
    properties
        Name          % Add a Name property to identify the brick
        Position
        Transform
        PlyFile
        Vertices
        MeshHandle 
    end
    
    methods
        
        function obj = Brick(name, position, transform, plyfile)
            if nargin > 0
                obj.Name = name;
                obj.Position = position;
                obj.Transform = transform;
                obj.PlyFile = plyfile;
                obj = obj.loadVertices();
            end
        end
        
        function obj = loadVertices(obj)
            % Load the mesh and get vertices, this will help treat the
            % brick as an object that could be attached/picked up by the
            % end-effector
            obj.MeshHandle = PlaceObject(obj.PlyFile, obj.Position);
            obj.Vertices = get(obj.MeshHandle, 'Vertices');
        end
        
        function obj = updateVertices(obj, newTransform)
            % Update vertices with a new transformation
            transformedVertices = [obj.Vertices, ones(size(obj.Vertices, 1), 1)] * newTransform';
            set(obj.MeshHandle, 'Vertices', transformedVertices(:, 1:3));
        end
        
        function plotBrick(obj)
            obj.MeshHandle = PlaceObject(obj.PlyFile, obj.Position);
            obj.updateVertices(obj.Transform);
        end

    end
end






