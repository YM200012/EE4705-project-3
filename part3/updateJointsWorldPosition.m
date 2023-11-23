%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta,mat)

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
    

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 
T_0 = cell(1,nJoints);


T_total = eye(4);

for k = 1:nJoints
        a = mat{k}(1, 4);
        alpha = atan2(mat{k}(2, 1), mat{k}(1, 1));
        d = mat{k}(3, 4);
        theta_k = theta(k);

        % Calculate the transformation matrix using the PoE formula
        A_k = [cos(theta_k), -sin(theta_k), 0, a;
               sin(theta_k) * cos(alpha), cos(theta_k) * cos(alpha), -sin(alpha), -d * sin(alpha);
               sin(theta_k) * sin(alpha), cos(theta_k) * sin(alpha), cos(alpha), d * cos(alpha);
               0, 0, 0, 1];
        B_k = [cos(theta_k),-sin(theta_k) * cos(alpha),sin(theta_k) * sin(alpha),a*cos(theta_k);
               sin(theta_k),cos(theta_k) * cos(alpha),-cos(theta_k) * sin(alpha),a*sin(theta_k);
               0,sin(alpha),cos(alpha),d;
               0,0,0,1];

        

        % Update the total transformation matrix
        T_total = T_total * B_k;

        % Store the transformation matrix for the kth joint
        T{k} = T_total;

        % Calculate the joint's world coordinates
        temp = T_total * [0; 0; 0; 1];
        X(k, :) = temp';
    
    
end
    
end