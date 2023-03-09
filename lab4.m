% Constants
l1 = 0.3;
l2 = 0.2;
l3 = 0.08;
s = 0.5;
H = 0.7;
R = 0.04;
h = 0.05;
s_p_h = [R, 0, h, 1]';
theta0 = 0;
alfa0 = 0;

% Variables, change for task 1a or 1b
task = 'b';

if task == 'a'
    theta1 = 0;
    theta2 = 0;
    b3 = 0.3;
    theta4 = 0;
elseif task == 'b'
    theta1 = 70*pi/180;
    theta2 = -50*pi/180;
    b3 = 0.4;
    theta4 = 80*pi/180;
end

% DH-parameters
a = [l1, l2, 0, 0];
b = [0, 0, b3, l3];
alfa = [0, pi, 0, 0];
theta = [theta1, theta2, 0, theta4];

% r-vectors
r = zeros(3,4); % Init
r_0 = [s, 0, H]';
r(:,1) = [l1*cos(theta(1)), l1*sin(theta(1)), 0]';
r(:,2) = [l2*cos(theta(2)), l2*sin(theta(2)), 0]';
r(:,3) = [0, 0, b3]';
r(:,4) = [0, 0, l3]';

% First rotation matrix
rotation_matrix_0 = [ cos(theta0), -sin(theta0)*cos(alfa0), sin(theta0)*sin(alfa0) ;
                      sin(theta0), cos(theta0)*cos(alfa0),  -cos(theta0)*sin(alfa0) ;
                      0,           sin(alfa0),              cos(alfa0) ];

% All other rotation matrixes
rotation_matrix = zeros(3,3,4); % Init
for i = 1:4
    rotation_matrix(:,:,i) = [ cos(theta(i)), -sin(theta(i))*cos(alfa(i)), sin(theta(i))*sin(alfa(i)) ;
                               sin(theta(i)), cos(theta(i))*cos(alfa(i)),  -cos(theta(i))*sin(alfa(i)) ;
                               0,             sin(alfa(i)),                cos(alfa(i)) ];
end

% First T-matrix
T_0 = [ rotation_matrix_0, r_0 ;
        0, 0, 0, 1 ];

% All other rotation matrixes
T = zeros(4,4,4); % Init
for i = 1:4
    T(:,:,i) = [ rotation_matrix(:,:,i), r(:,i) ;
                 0, 0, 0, 1 ];
end

r_p_h = T_0*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*s_p_h;

P_position = r_p_h(1:3);

%%

% Additional variables
theta1_dot = -pi/6;
theta2_dot = 65*pi/180;
b3_dot = 0.2;
theta4_dot = pi/12;

% eta_dot-vector
eta_dot = [theta1_dot, theta2_dot, b3_dot, theta4_dot]';

% Partial T-matrixes
T_1_0 = T_0;
T_2_0 = T_1_0*T(:,:,1);
T_3_0 = T_2_0*T(:,:,2);
T_4_0 = T_3_0*T(:,:,3);

% z-vectors
z_1 = T_1_0(1:3, 3);
z_2 = T_2_0(1:3, 3);
z_3 = T_3_0(1:3, 3);
z_4 = T_4_0(1:3, 3);

% rho-vectors
rho_1 = P_position(1:3) - T_1_0(1:3, 4);
rho_2 = P_position(1:3) - T_2_0(1:3, 4);
rho_3 = P_position(1:3) - T_3_0(1:3, 4);
rho_4 = P_position(1:3) - T_4_0(1:3, 4);

% J-matrixes, r, r, p ,r
J_1 = [ cross(z_1, rho_1) ; z_1 ];
J_2 = [ cross(z_2, rho_2) ; z_2 ];
J_3 = [ z_3 ; zeros(3,1) ];
J_4 = [ cross(z_4, rho_4) ; z_4 ];

% Full J-matrix
J = [J_1, J_2, J_3, J_4];

combined_velocity_ang_velocity = J*eta_dot;

P_velocity = combined_velocity_ang_velocity(1:3);
angular_velocity_body_4 = combined_velocity_ang_velocity(4:6);
