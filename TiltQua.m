% quadrotor_tilt_simulation.m
% Mô phỏng quadrotor tilt hai trục với gyroscopic actuation
% Dựa trên luận văn Pau Seguí Gascó (2012)
% Sử dụng Symbolic Math Toolbox để derive dynamics
% Giải ODE bằng ode45 cho 6DOF

close all; 
clc;

%% Bước 1: Định nghĩa tham số (từ luận văn, Table 3.5 và experiments)
% Physical parameters
m = 1.5;        % Mass (kg), từ mass breakdown (Table 3.5)
g = 9.81;       % Gravity (m/s^2)
rho = 1.225;    % Air density (kg/m^3)
R = 0.15;       % Propeller radius (m, 12x6 prop)
A = pi * R^2;   % Propeller area (m^2)
Ct = 0.013;     % Thrust coefficient (từ 4.3)
Cq = 0.0013;    % Torque coefficient (từ 4.3)
Ixx = 0.1;      % Moment of inertia (kg*m^2, từ CAD, Table 3.5)
Iyy = 0.1;      
Izz = 0.2;      
Ip = 0.001;     % Propeller inertia (kg*m^2)
l = 0.25;       % Arm length (m, giả định từ thiết kế)

% Actuator parameters (từ 4.4 và 4.5)
Km = 9.19;      % Motor gain (từ 4.4)
Tm = 0.16;      % Motor time constant
delay_m = 0.035; % Motor delay
Ks = 1228;      % Servo gain (từ 4.5)
Ds = 49.18;     % Servo damping

% Initial states: [x y z phi theta psi dx dy dz p q r]
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%% Bước 2: Derive Equations of Motion (từ 4.2 Rigid Body và 4.3 Aero)
% Symbolic variables
syms x y z phi theta psi dx dy dz p q r eta gamma deta dgamma Omega u1 u2 u3 u4 t

% Rotation matrix (body to inertial)
R_matrix = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
            -sin(theta),         sin(phi)*cos(theta),                        cos(phi)*cos(theta)];

% Thrust and torque per rotor (từ 4.3)
T = rho * A * (Omega * R)^2 * Ct;  % Thrust
Q = rho * A * (Omega * R)^2 * R * Cq;  % Torque

% Tổng thrust (cho hover)
U1 = rho * A * R^2 * Ct * (u1 + u2 + u3 + u4);  % Tổng thrust từ 4 rotors

% Gyroscopic moments cho một rotor (từ 4.2)
omega_total = [cos(eta)*p - r*sin(eta) + dgamma;
               sin(gamma)*(cos(eta)*r + p*sin(eta)) + cos(gamma)*(q + deta);
               cos(gamma)*(cos(eta)*r + p*sin(eta)) + Omega - sin(gamma)*(q + deta)];

alpha_total = diff(omega_total, t);  % Symbolic derivative

I_prop = diag([Ip/2, Ip/2, Ip]);  % Propeller inertia (đối xứng)

M_gyro = I_prop * alpha_total + cross(omega_total, I_prop * omega_total);

% Chuyển gyro moment về body frame (giả định rotor 1)
R_3to1 = [cos(gamma)*cos(eta), sin(gamma), cos(gamma)*sin(eta);
          -sin(gamma)*cos(eta), cos(gamma), -sin(gamma)*sin(eta);
          -sin(eta),            0,           cos(eta)];
M_gyro_body = R_3to1 * (-M_gyro);  % Reaction moment

% Tổng moments (cho cả 4 rotors, giả định đối xứng)
tau = [l * rho * A * R^2 * Ct * (u4 - u2);  % Roll
       l * rho * A * R^2 * Ct * (u3 - u1);  % Pitch
       rho * A * R^3 * Cq * (u1 - u2 + u3 - u4)];  % Yaw
tau = tau + 4 * M_gyro_body;  % Thêm gyroscopic moments

% Equations of motion (6DOF, từ 4.6)
acc = (1/m) * R_matrix * [0; 0; -U1] + [0; 0; g];  % Translational (bỏ qua drag nhỏ)

% Rotational (Euler)
p_dot = (q*r*(Iyy - Izz) + tau(1)) / Ixx;
q_dot = (p*r*(Izz - Ixx) + tau(2)) / Iyy;
r_dot = (p*q*(Ixx - Iyy) + tau(3)) / Izz;

% Euler angle rates
phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
theta_dot = cos(phi)*q - sin(phi)*r;
psi_dot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

% State vector
state_vec = [x; y; z; phi; theta; psi; dx; dy; dz; p; q; r];
f = [dx; dy; dz; phi_dot; theta_dot; psi_dot; acc(1); acc(2); acc(3); p_dot; q_dot; r_dot];

%% Bước 3: Generate MATLAB Function cho ODE
%matlabFunction(f, 'File', 'quad_dynamics_tilt.m', 'Vars', {t, state_vec, u1, u2, u3, u4, eta, gamma, deta, dgamma, Omega});

%% Bước 4: Mô phỏng bằng ode45
tspan = [0 10];  % Thời gian mô phỏng 10s

% Control inputs (hover với Omega^2 ~ 10000 rad^2/s^2 per rotor)
u = @(t) [10000; 10000; 10000; 10000];  % [u1 u2 u3 u4]
% Tilt inputs (small oscillations để thấy gyro effects)
tilt = @(t) [0.1*sin(t); 0.1*cos(t); 0.05; 0.05; 5000];  % [eta gamma deta dgamma Omega]

% ODE function wrapper
odefun = @(t, x) quad_dynamics_tilt_wrapper(t, x, u, tilt);

[t, x] = ode45(odefun, tspan, x0);

% Wrapper function to handle array indexing
function dxdt = quad_dynamics_tilt_wrapper(t, x, u_func, tilt_func)
    u_val = u_func(t);
    tilt_val = tilt_func(t);
    dxdt = quad_dynamics_tilt(t, x, u_val(1), u_val(2), u_val(3), u_val(4), ...
                             tilt_val(1), tilt_val(2), tilt_val(3), tilt_val(4), tilt_val(5));
end

%% Bước 5: Plot kết quả
figure;
subplot(3,1,1); plot(t, x(:,1:3)); legend('x', 'y', 'z'); title('Vị trí (m)'); xlabel('Thời gian (s)'); ylabel('Vị trí');
subplot(3,1,2); plot(t, x(:,4:6)*180/pi); legend('phi', 'theta', 'psi'); title('Góc Euler (deg)'); xlabel('Thời gian (s)'); ylabel('Góc');
subplot(3,1,3); plot(t, x(:,10:12)*180/pi); legend('p', 'q', 'r'); title('Vận tốc góc (deg/s)'); xlabel('Thời gian (s)'); ylabel('Vận tốc góc');

%% Bước 6: Thêm Actuator Dynamics (tùy chọn, từ 4.4-4.5)
% Motor transfer function: G_motor = exp(-0.035s) * 9.19/(1 + 0.16s)
% Servo transfer function: G_servo = 1228.05/(s^2 + 49.18s + 1228.05)
% Có thể thêm bằng cách dùng 'tf' và 'lsim' để mô phỏng inputs trước khi vào odefun