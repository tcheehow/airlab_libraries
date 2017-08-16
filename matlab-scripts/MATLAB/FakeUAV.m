classdef FakeUAV
    %FAKEUAV Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensorOrientation
        uavOrientation %yaw, pitch, yaw
        uavPos
    end
    
    methods
        function obj = FakeUAV(orient, pos)
            if nargin == 2
                obj.sensorOrientation = ...
                    [1, -pi/4;
                    1, -pi/2;
                    1, -3*pi/4;
                    1, 3*pi/4;
                    1, pi/2;
                    1, pi/4];
                obj.uavOrientation = orient;
                obj.uavPos = pos;
            else
                error('requires two arguments [yaw, pitch, roll] and [x y z s]')
            end
        end
        
        function cart = getRayCartesian_body(obj)
            polar   = obj.sensorOrientation;
            d       = size(polar);
            r       = polar(:,1);
            th      = polar(:,2);
            x       = r.*cos(th);
            y       = r.*sin(th);
            cart    = [x, y, zeros(d(1),1)];
            cart    = cart2hom(cart);
            % cart = [-y, x, zeros(d(1),1)];
        end
        
        function cart = getRayCartesian_world(obj)
            polar   = obj.sensorOrientation;
            d       = size(polar);
            r       = polar(:,1);
            th      = polar(:,2);
            x       = r.*cos(th);
            y       = r.*sin(th);
            cart    = [x, y, zeros(d(1),1)];
            cart    = cart2hom(cart);
            % cart = [-y, x, zeros(d(1),1)];
        end
        
        function rotm = getRotMatrix(obj)
            rotm    = rpy(obj.uavOrientation, obj.uavPos);
        end
        
    end
    
end

