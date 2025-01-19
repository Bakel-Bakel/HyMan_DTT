% Define DH parameters for the planar arm
planar_arm = [
    0,   0,   pi/2,  -pi/2,  0;   % Link 1
    0, 750,   0,    0, 0;   % Link 2
    0, 750,   0,    0,   0;    % Link 3
    0, 750,   0,    0,   0;    % Link 4
    0, 750,   0,    0,   0;    % Link 5
];

% Define DH parameters for the spatial arm
spatial_arm = [
    0,   0,   0,    0, 0;   % Link 6
  600,   0,   0, -pi/2, 0;  % Link 7
    0,   0,   0,  pi/2,   0;   % Link 8
  0,    0,   0,    0,   0;    % Link 9
    0,   0,   0, -pi/2,   0;   % Link 10
    0,   0,   0,  pi/2,   0;   % Link 11
  526,   0,   0,    0,   0;    % Link 12
];

% Combine planar and spatial arms
dh_table = [planar_arm; spatial_arm];

% Initialize identity matrix for cumulative transformation
T = eye(4);

% Iterate over each link and compute the transformation
for i = 1:size(dh_table, 1)
    d = dh_table(i, 1);
    a = dh_table(i, 2);
    alpha = dh_table(i, 3);
    theta = dh_table(i, 4) + dh_table(i, 5);  % Add the offset
    
    % Compute the transformation matrix for this link
    A = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),             d;
        0,           0,                      0,                      1;
    ];
    
    % Multiply to get the cumulative transformation
    T = T * A;
end

% Display the final transformation matrix
disp('Forward Kinematics Transformation Matrix (T):');
disp(T);
