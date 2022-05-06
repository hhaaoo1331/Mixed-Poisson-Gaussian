function [u_update,k,SSIM,PSNR,t,min_w]= tvic_Huber_mix_denoise_PBCA(f_true,f,H,iter,epsilon,lambda1,lambda2,p1,p2,alpha,gamma)

%   A Proximal Bilinear Constraint based ADMM (PBCA) algorithm for image restoration with mixed Poisson-Gaussian noise.
% INPUTS: 
%    f_true -  name of the clean image.
%    f  -  name of the noise image.
%    H  -  name of the fuzzy operator, e.g. H =fspecial('gaussian',7,3).
%    p1 - name of the penalty parameters of the PBCA algorithm.
%    p2 - name of the penalty parameters of the PBCA algorithm.
%    alpha - name of the parameters in the positive definite matrix P.
%    gamma - name of the parameters in Huber regular function.
%    lambda1 - regularization parameter.
%    lambda2 - regularization parameter.
%    epsilon - The error value satisfied by the outer loop iteration.
%    iter - maximum number of iterations.
% OUPUTS:
%    u_update - name of the reconstructed image.
%    t - The time consumed by the algorithm.
%    k - Number of iteration steps of PBCA algorithm.
%    min_w - name of the minimum value of w during iteration.

[m,n]=size(f_true);

%Initialization primal variable.
u = zeros(m,n);
v = zeros(m,n);
w = ones(m,n);
y = zeros(m,2*n);

%Initialization dual variable.
d1 = zeros(m,n);
d2 = zeros(m,2*n);

k =1;
tic
while k <= iter
    
      %u-subproblem.   
      c1 = 1/alpha;
      A1 = alpha.*(c1.*u - p1.*imfilter(imfilter(u, H), rot90(H,2)) -p2.*Dt(D(u)) - imfilter(d1, rot90(H,2)) + p1.*imfilter(w.*v, rot90(H,2)) - Dt(d2) + p2.*Dt(y));
      u_update = proj_bound(A1,0,1);
      
      %v-subproblem.   
      v_update1 = (lambda1.*f + lambda2.*log(w) + p1.*w.*imfilter(u_update, H))./(lambda1 + p1.*w.^2);
      v_update = max(v_update1,1e-5);
      
      %w-subproblem.   
      A2 = lambda2 - p1.*(imfilter(u_update, H) + d1./p1);
      w_update = (-A2 + sqrt(A2.^2 + 4*p1*lambda2.*v_update))./(2*p1.*v_update);
      min_w(k) = min(min(w_update));
      
      %y-subproblem.   
      A3 = D(u_update) + d2./p2;
      y_update = prox_gamma_huber(A3,1/p2,gamma);   

      %Multiplier update.
      d1_update = d1 + p1.*(imfilter(u_update, H) - w_update.*v_update);
      d2_update = d2 + p2.*(D(u_update) - y_update);
          
      
      SSIM = ssimCompt(255*f_true,255*u_update);
      PSNR = 20*log10(1/sqrt(mean((u_update(:)-f_true(:)).^2)));
        
   
    if  norm(u_update(:)-u(:))/norm(u(:)) <= epsilon
        break;
    else
        u = u_update;  
        v = v_update;  
        w = w_update;  
        y = y_update; 
        d1 = d1_update;
        d2 = d2_update;  
        k = k+1;
    end
end
 toc   
 t=toc;  
end
