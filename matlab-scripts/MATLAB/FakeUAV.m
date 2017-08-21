classdef FakeUAV
    %FAKEUAV Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sensorOffset % body
        sensorOffsetWorld
        troneReadings
        sensorOrientation % polar
        sensorOrientationXY % cartesian
        uavOrientation %yaw, pitch, yaw
        uavPos
        uavROTM
    end
    
    methods
        function obj = FakeUAV(orient, pos)
            if nargin == 2
                obj.sensorOrientation = ...
                    [1, -pi/4;
                    1, -pi/2;
                    1, -pi/2;
                    1, pi/2;
                    1, pi/2;
                    1, pi/4];
                obj.uavOrientation      = orient;
                obj.uavPos              = pos;
                obj.uavROTM             = rpy(obj.uavOrientation, obj.uavPos);
                obj.sensorOrientationXY = obj.sensorPol2Cart();
                obj.sensorOffset        = ...
                    [0.2256, -0.1741 0;
                    0.1739 -0.1915 0;
                    -0.1739 -0.1915 0;
                    -0.1739 0.1915 0;
                    0.1739 0.1915 0;
                    0.2256 0.1741 0];
                obj.sensorOffsetWorld = cart2hom(obj.sensorOffset);
                for i=1:length(obj.sensorOffsetWorld)
                    v = obj.sensorOffsetWorld(i,:)';
                    v = obj.uavROTM * v;
                    obj.sensorOffsetWorld(i,:) = v';
                end
            else
                error('requires two arguments [yaw, pitch, roll] and [x y z s]')
            end
        end
        
        function cart = sensorPol2Cart(obj)
            polar   = obj.sensorOrientation;
            cart    = obj.rotatePol2Cart(polar);
        end
        
        function cart = rotatePol2Cart(obj, polar)
            d       = size(polar);
            r       = polar(:,1);
            th      = polar(:,2);
            x       = r.*cos(th);
            y       = r.*sin(th);
            cart    = [x, y, zeros(d(1),1)];
            cart    = cart2hom(cart);
        end
        
        function cart = getSensorXY_World(obj)            
            % body-frame xy-coordinate of trone ray
            % (x6,y6)o     o(x1,y1)
            % (x5,y5)o     o(x2,y2)
            % (x4,y4)o     o(x3,y3)
            
            sensor_cart             = obj.sensorOrientationXY;         
            
            % rotate all sensor ray to world frame using known rotation
            % matrix and translation between the uav frame origin and the
            % world frame origin
            
            for i=1:length(sensor_cart)
                v = sensor_cart(i,:)';
                rotm = obj.uavROTM;
                rotm(1:3,end) = rotm(1:3,end) + obj.sensorOffsetWorld(i,1:3)'; 
                v = rotm * v;
                sensor_cart(i,:) = v';
            end
            
            cart = sensor_cart;
            
        end
        
        function rotm = getRotMatrix(obj)
            rotm    = obj.uavROTM;
        end
        
    end
    
end

