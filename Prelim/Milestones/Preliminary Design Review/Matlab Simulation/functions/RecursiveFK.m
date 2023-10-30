function [T_ITN, w_NNwI, J_N, dot_J_N] = RecursiveFK(T_ITP, T_PTN, w_PPwI, J_P, dot_J_P, r_PPrN, I_hat_N, I_tilde_N, dot_gamma)

% not sure which vars are supposed to be input and which are calculated
% from inputs.

w_PPwI = J_P(1:3, :)*dot_gamma;

T_ITN = T_ITP * T_PTN;
w_NNwP = I_hat_N * dot_gamma;
dot_r_PPrN = I_tilde_N * dot_gamma;

J_N = [             T_PTN.', zeros(3,3);
      -T_ITP * skew(r_PPrN), eye(3)] * J_P...
      +...
      [        I_hat_N;
      T_ITP*I_tilde_N];

w_NNwI = J_N(1:3, :)*dot_gamma;

dot_J_N = [                                    -skew(w_NNwP)*T_PTN.', zeros(3,3);
           -T_ITP*(skew(w_PPwI)*skew(r_PPrN) + skew(dot_r_PPrN)), zeros(3,3)] * J_P...
           +...
           [                 T_PTN.',        zeros(3,3);
           -T_ITP * skew(r_PPrN), eye(3)] * dot_J_P...
           +...
           [zeros(3,length(dot_gamma));
           T_ITP*skew(w_PPwI)*I_tilde_N];
           
end

