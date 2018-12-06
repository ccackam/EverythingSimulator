function [K] = gains_SS(t_r,zeta,p_i,A,B,C_r)

    w_n = 2.2./t_r;
    
    poles = [];
    for i = 1:length(t_r)
        Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
        poles = [poles;roots(Delta)];
    end
    
    if p_i ~= 0
        poles = [poles;p_i];
        A = [A,zeros(length(A),1);-C_r,0];
        B = [B;0];
    end
    
    C_A = zeros(size(A));
    for n = 1:length(A)
        C_A(:,n) = A^(n-1)*B;
    end
    
    if rank(C_A) ~= length(A)
        error('System not controlable!')
    end
    
    K.K = place(A,B,poles);
    
%     if length(A) == 4
%         K = [-0.3933,93.1165,-0.2831,8.3788]
%     end
    
    if p_i == 0
        K.k_r = -1./(C_r*((A-B*K.K)^-1)*B);
    elseif p_i ~= 0
        K.k_i = K.K(end);
        K.K = K.K(1:end-1);
    end
    
    % k_r = -30
    
end