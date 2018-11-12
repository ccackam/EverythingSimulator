function [K,k_r] = gains_SS(t_r,zeta,A,B,C_r)

    w_n = 2.2./t_r;
    
    poles = [];
    for i = 1:length(t_r)
        Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
        poles = [poles;roots(Delta)];
    end
    
    C_A = zeros(size(A));
    for n = 1:length(A)
        C_A(:,n) = A^(n-1)*B;
    end
    
    if rank(C_A) ~= length(A)
        error('System not controlable!')
    end
    
    K = place(A,B,poles);
    
%     if length(A) == 4
%         K = [-0.3933,93.1165,-0.2831,8.3788]
%     end
    
    k_r = -1./(C_r*((A-B*K)^-1)*B)
    
    k_r = -30
    
end