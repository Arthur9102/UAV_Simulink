function B = B_matrix()
    % Tham số từ luận văn (ước lượng, cần điều chỉnh theo thực tế)
    mu = 0.0001;        % Hệ số thrust
    kappa = 0.001;      % Hệ số torque
    J = 110.9e-6;       % Moment quán tính (kg·m²)
    Omega = 100;        % Tốc độ quay propeller (rad/s)
    tr = 0.1;           % Rise time servo (s)
    L = 0.25;           % Chiều dài arm (m)
    H = 0.05;           % Chiều cao hub (m)
    T0 = 1.5;           % Thrust tại hover (N)

    % Submatrix cho Standard Arm (4x3)
    B_std = [-L*mu, -J*Omega/tr, H*T0;
             H*T0, L*T0, J*Omega/tr;
             kappa, 0, 0;
             mu, 0, 0];

    % Ma trận B đầy đủ (4x12) cho 4 arms
    % Arm 1, 2, 3, 4 với rotation và dấu ± cho kappa
    B = zeros(4, 12);
    % Arm 1 (Standard)
    B(:, 1:3) = B_std;
    % Arm 2 (Rotate 90 deg, ví dụ: đảo dấu ở hàng 1, 2)
    B(:, 4:6) = [-H*T0, -L*T0, J*Omega/tr;
                 -L*mu, -J*Omega/tr, 0;
                 kappa, 0, 0;
                 mu, 0, 0];
    % Arm 3 (Rotate 180 deg, ví dụ: đảo dấu)
    B(:, 7:9) = [L*mu, J*Omega/tr, H*T0;
                 -H*T0, -L*T0, J*Omega/tr;
                 kappa, 0, 0;
                 mu, 0, 0];
    % Arm 4 (Rotate 270 deg)
    B(:, 10:12) = [H*T0, L*T0, -J*Omega/tr;
                   L*mu, J*Omega/tr, 0;
                   -kappa, 0, 0;
                   mu, 0, 0];
end