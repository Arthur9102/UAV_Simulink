function u = weighted_pseudoinverse(v, B, W)
    % Input: v (4x1), B (4x12), W (12x12 diagonal)
    % Output: u (12x1)
    
    % Tính weighted pseudoinverse
    % Winv = inv(W); % Ma trận nghịch đảo trọng số
    P_w = W \ B' * pinv(B *W * B'); % Sử dụng pinv để ổn định
    u = P_w * v;
    
    % Xử lý ràng buộc (optional, ví dụ: u trong [-1, 1])
    u = max(-1, min(1, u));
end