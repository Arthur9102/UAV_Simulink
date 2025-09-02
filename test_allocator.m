v = [0.01; 0.02; 0.05; 1.5]; % Lệnh ảo
B = B_matrix();
W = diag([0.994, 0.003, 0.003, 0.994, 0.003, 0.003, ...
          0.994, 0.003, 0.003, 0.994, 0.003, 0.003]);
u = weighted_pseudoinverse(v, B, W);
disp(u);
disp(B*u - v); % Kiểm tra lỗi