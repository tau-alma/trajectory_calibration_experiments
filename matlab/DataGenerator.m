classdef DataGenerator < handle
    %DATAGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        T_S2_S1
        T_S1_W1
        T_S2_W2
        numPoints
        normal
    end
    
    properties (SetAccess = private)
        dt
        x
        y
        z
        roll
        pitch
        yaw
        T_W1_W2
        XX
        YY
        ZZ
    end
    
    methods
        function obj = DataGenerator()
            %DATAGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function generateRandomTrajectory(obj, options)
            %GENERATERANDOMTRAJECTORY Generates a random base trajectory
            arguments
                obj
                options.flatness double = 1e-1
                options.numPoints {mustBeInteger} = 100
            end
            
            obj.numPoints = options.numPoints;
            num_points = options.numPoints + 1;
            
            % The actual transformation
            R = quat2rotm(randrot());
            t = -0.5 + 1*rand(3,1);

            obj.T_S2_S1 = [R, t; 0, 0, 0, 1];
            
            % Generate first trajectory
            obj.dt = 0.1;
            v = 0.5;
            pose = zeros(3, num_points);
            pose(2,1) = 0.5;

            np = round(num_points / 5);
            w = -2 + 4*rand(1, np);
            w = repelem(w, 5);
            if length(w) > num_points
                w = w(1:num_points);
            elseif length(w) < num_points
                w(end+1) = w(end);
            end

            for k = 2:num_points
                pose(:, k) = motionModel(pose(:, k-1), [v, w(k), obj.dt]);
            end
            
            % Create terrain
            [obj.XX, obj.YY] = ndgrid(pose(1,:), pose(2,:));
            obj.ZZ = options.flatness * peaks(obj.XX, obj.YY);

            [Nx, Ny, Nz] = surfnorm(obj.XX, obj.YY, obj.ZZ);
            n = [diag(Nx), diag(Ny), diag(Nz)]';
            % Make sure normal is pointing up
            n = n .* sign(n(3,:));
            
            obj.normal = n(:, 2:end);

            % Base trajectory
            % First pose is incorrect due to normals calculation
            obj.x = pose(1, 2:end);
            obj.y = pose(2, 2:end);
            z = diag(obj.ZZ);
            obj.z = z(2:end);
            obj.yaw = pose(3, 2:end);
            obj.pitch = angleFromPlane([1, 0, 0, 0], n(:, 2:end));
            obj.roll = angleFromPlane([0, 1, 0, 0], n(:, 2:end));

            % Random transition between world frames
            obj.T_W1_W2 = [angle2dcm(rand-rand, rand-rand, rand-rand, 'ZYX'), rand(3,1)-rand(3,1); 0,0,0,1];
        end
        
        function generateData(obj, options)
            %GENERATEDATA Generates simulated data for two sensors
            %   Uses the current trajectory
            arguments
                obj
                options.NoiseVar
                options.Scale
                options.Outliers
                options.Drift
            end
            
            if isfield(options, "NoiseVar")
                noise_var = options.NoiseVar;
            else
                noise_var = zeros(1,6);
            end
            if isfield(options, "Scale")
                scale = options.Scale;
            else
                scale = 1;
            end
            if isfield(options, "Outliers")
                insert_outliers = true;
                outliers = options.Outliers;
            else
                insert_outliers = false;
            end
            if isfield(options, "Drift")
                insert_drift = true;
                drift_rate = options.Drift;
            else
                insert_drift = false;
            end
            
            % Generate second trajectory based on first
            for k = 1:obj.numPoints
                T1 = [angle2dcm(obj.yaw(k), obj.pitch(k), obj.roll(k), 'ZYX'), [obj.x(k); obj.y(k); obj.z(k)]; 0,0,0,1];
                T2 = obj.T_W1_W2 * T1 * obj.T_S2_S1;

                d = noise_var .* randn(1,6);
                T1_noisy = T1 * [angle2dcm(d(6), d(5), d(4), 'ZYX'), d(1:3)'; 0,0,0,1];

                d = noise_var .* randn(1,6);
                T2_noisy = T2 * [angle2dcm(d(6), d(5), d(4), 'ZYX'), d(1:3)'; 0,0,0,1];

                obj.T_S1_W1.p{k} = T1_noisy(1:3, end);
                obj.T_S1_W1.R{k} = T1_noisy(1:3, 1:3);

                obj.T_S2_W2.p{k} = scale * T2_noisy(1:3, end);
                obj.T_S2_W2.R{k} = T2_noisy(1:3, 1:3);
            end

            if insert_drift
                ax = randi(3);
                relative_pose = cell(obj.numPoints-1,1);
                for k = 1:obj.numPoints-1
                    % get relative pose
                    relative_pose{k} = asT(obj.T_S2_W2.R{k}, obj.T_S2_W2.p{k}, 'inv') * ...
                            asT(obj.T_S2_W2.R{k+1}, obj.T_S2_W2.p{k+1});
                    % calculate absolute movement
                    movement = norm(relative_pose{k}(1:3, end));
                    % multiply by drift_rate to get amount of drift
                    drift = zeros(3,1);
                    drift(ax) = movement * drift_rate;
                    % add drift
                    relative_pose{k}(1:3,end) = relative_pose{k}(1:3,end) + drift;
                end
                for k = 2:obj.numPoints
                    T = asT(obj.T_S2_W2.R{k-1}, obj.T_S2_W2.p{k-1}) * relative_pose{k-1};
                    obj.T_S2_W2.R{k} = T(1:3,1:3);
                    obj.T_S2_W2.p{k} = T(1:3,end);
                end
            end

            if insert_outliers
                r = randi(obj.numPoints, outliers * obj.numPoints, 2);
                jumps = randomJumps(outliers * obj.numPoints);
                for i = 1:size(r,1)
                    affected = false(obj.numPoints,2);
                    affected(r(i,1):end,1) = true;
                    affected(r(i,2):end,2) = true;
                    for k = 1:obj.numPoints
                        if affected(k,1)
                            obj.T_S1_W1.p{k} = obj.T_S1_W1.p{k} + jumps(:,1,i);
                        end
                        if affected(k,2)
                            obj.T_S2_W2.p{k} = obj.T_S2_W2.p{k} + jumps(:,2,i);
                        end
                    end
                end
            end
        end
        
        function [tr_err, rot_err] = get_error(obj)
            [tr_err, rot_err] = calculateError(obj.T_S1_W1, obj.T_S2_W2, obj.T_S2_S1);
        end
        
        function print_error(obj)
            [tr_err, rot_err] = obj.get_error();
            fprintf('Translational error: %f Rotational error: %f\n', tr_err, rot_err);
        end
        
        function save(obj, path)
            status = mkdir(path);
            saveTrajectory(obj.T_S1_W1, obj.dt, fullfile(path,'T_S1_W1.txt'))
            saveTrajectory(obj.T_S2_W2, obj.dt, fullfile(path,'T_S2_W2.txt'))
            T.p{1} = obj.T_S2_S1(1:3, end);
            T.R{1} = obj.T_S2_S1(1:3, 1:3);
            saveTrajectory(T, obj.dt, fullfile(path,'T_S2_S1.txt'))
        end
        
        function plot(obj)
            figure;
            surf(obj.XX, obj.YY, obj.ZZ);
            xlabel('x'), ylabel('y'), zlabel('z')
            hold on
            grid on
            axis equal
            for k = 1:obj.numPoints
                T2 = obj.T_W1_W2^-1 * [obj.T_S2_W2.R{k}, obj.T_S2_W2.p{k}; 0,0,0,1];
                plot3(obj.T_S1_W1.p{k}(1), obj.T_S1_W1.p{k}(2), obj.T_S1_W1.p{k}(3), '.r');
                plot3(T2(1,end), T2(2,end), T2(3,end), '.b');
                
                normal = obj.T_S1_W1.p{k} + obj.normal(:, k);
                x = [obj.T_S1_W1.p{k}(1), normal(1)];
                y = [obj.T_S1_W1.p{k}(2), normal(2)];
                z = [obj.T_S1_W1.p{k}(3), normal(3)];
                plot3(x, y, z, '-r');
            end
        end
        
        function cost_plot(obj)
            costPlot(@cost, obj.T_S1_W1, obj.T_S2_W2, obj.T_S2_S1);
        end
        
        function b = data_generated(obj)
            if length(obj.T_S1_W1) == 1
                b = true;
            else
                b = false;
            end
        end
    end
end

%% Function definitions
function pose = motionModel(pose, control)
% VELOCITYMOTIONMODEL Motion model for velocity control
x = pose(1);
y = pose(2);
th = pose(3);
v = control(1);
w = control(2);
dt = control(3);
% Take into account the special case w=0
if w==0
    dtv = dt*v;
    pose = [x+dtv*cos(th); y+dtv*sin(th); th];
else
    r = v/w;
    th_dtw = th+dt*w;
    pose = [x+r*sin(th_dtw)-r*sin(th);
            y-r*cos(th_dtw)+r*cos(th);
            wrapToPi(th_dtw)];
end
end

function a = angleFromPlane(plane_coefficients, line_direction)
a = zeros(1, size(line_direction, 2));
plane_normal = plane_coefficients(1:3);
for k = 1:length(a)
  s = sign(dot(plane_normal, line_direction(:,k)));
  a(k) = s * asin(abs(dot(plane_normal, line_direction(:,k))) ...
       / (norm(plane_normal) * norm(line_direction(:,k))));
end
end

function saveTrajectory(T, dt, filename)
[x,y,z,qx,qy,qz,qw] = cellfun(@extractRow, T.p, T.R);
t = dt * (0:length(x)-1);
M = horzcat(t',x',y',z',qx',qy',qz',qw');
writematrix(sprintf('# Generated using MATLAB %s on %s\n# timestamp tx ty tz qx qy qz qw', mfilename, date),filename);
writematrix(M,filename,'Delimiter',' ', 'WriteMode','append');
end

function [x,y,z,qx,qy,qz,qw] = extractRow(p, R)
x = p(1);
y = p(2);
z = p(3);
quat = rotm2quat(R);
qx = quat(2);
qy = quat(3);
qz = quat(4);
qw = quat(1);
end

function jump = randomJumps(n)
jump = zeros(3,2,n);
sigma = 0.02 * eye(3);
R = chol(sigma);
jump(:,1,:) = (randn(n,3)*R)';
jump(:,2,:) = (randn(n,3)*R)';
end

function T = asT(varargin)
if nargin < 3
    inv = false;
elseif strcmp(varargin{3},'inv')
    inv = true;
else
    error("Unknown arg '%s'",varargin{3})
end
R = varargin{1};
p = varargin{2};
if inv
    T = [R' -R'*p; 0 0 0 1];
else
    T = [R p; 0 0 0 1];
end
end

function [tr_error, rot_error] = calculateError(T_S1_W1, T_S2_W2, T_S2_S1)
num_points = length(T_S1_W1.R);

T10i = asT(T_S1_W1.R{1}, T_S1_W1.p{1}, 'inv');
T1 = zeros(4, 4, num_points);
for i = 1:num_points
    T1(:,:,i) = T10i * asT(T_S1_W1.R{i}, T_S1_W1.p{i});
end

T20i = asT(T_S2_W2.R{1}, T_S2_W2.p{1}, 'inv');
T2 = zeros(4, 4, num_points);
for i = 1:num_points
    T2(:,:,i) = T20i * asT(T_S2_W2.R{i}, T_S2_W2.p{i});
end

tr_error = 0;
rot_error = 0;
X = T_S2_S1;
for i = 1:num_points
    Td = T1(:,:,i)*X - X*T2(:,:,i);
    tr_error = tr_error + norm(Td(1:3,end));
    
    R = (X(1:3, 1:3) * T2(1:3,1:3,i)) \ (T1(1:3,1:3,i) * X(1:3, 1:3));
    rot_error = rot_error + rad2deg(norm(rotationMatrixToVector(R)));
end
end

function costPlot(cost, T_S1_W1, T_S2_W2, T)
[T1, T2] = relativeTrajectory(T_S1_W1, T_S2_W2);
figure;
[yaw, pitch, roll] = dcm2angle(T(1:3,1:3), 'ZYX');
r = rad2deg([roll, pitch, yaw]);
x = T(1:3,end);
params = [x(1), x(2), x(3), r(1), r(2), r(3)];
labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];

for i = 1:length(params)
    par = params;
    if i < 3
        a = 1;
    else
        a = 10;
    end
    var = linspace(params(i)-a, params(i)+a, 11);
    c = [];
    for v = var
        par(i) = v;
        p = par(1:3);
        r = deg2rad(par(4:6));
        R = angle2dcm(r(3), r(2), r(1), 'ZYX');
        c(end+1) = cost(T1, T2, R, p');
    end
    subplot(2,3,i);
    plot(var, c)
    xline(params(i), 'r--')
    xlabel(labels(i))
    ylabel('Cost')
end
end

function cost = cost(T1, T2, R, p)
cost = 0;
X = asT(R,p);
for i = 1:size(T1, 3)
    Td = T1(:,:,i)*X - X*T2(:,:,i);
    cost = cost + norm(Td);
end
end

function [T1, T2] = relativeTrajectory(T_S1_W1, T_S2_W2)
num_points = length(T_S1_W1.R);

T10i = asT(T_S1_W1.R{1}, T_S1_W1.p{1}, 'inv');
T1 = zeros(4, 4, num_points);
for i = 1:num_points
    T1(:,:,i) = T10i * asT(T_S1_W1.R{i}, T_S1_W1.p{i});
end

T20i = asT(T_S2_W2.R{1}, T_S2_W2.p{1}, 'inv');
T2 = zeros(4, 4, num_points);
for i = 1:num_points
    T2(:,:,i) = T20i * asT(T_S2_W2.R{i}, T_S2_W2.p{i});
end
end
