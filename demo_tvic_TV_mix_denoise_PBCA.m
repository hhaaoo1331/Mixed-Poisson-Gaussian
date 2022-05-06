clc,clear

%   A Proximal Bilinear Constraint based ADMM (PBCA) algorithm for image restoration with mixed Poisson-Gaussian noise.
% INPUTS: 
%    f -  name of the clean image.
%    g  -  name of the noise image.
%    H  -  name of the fuzzy operator, e.g. H =fspecial('gaussian',7,3).
%    p1 - name of the penalty parameters of the PBCA algorithm.
%    p2 - name of the penalty parameters of the PBCA algorithm.
%    alpha - name of the parameters in the positive definite matrix P.
%    lambda1 - regularization parameter.
%    lambda2 - regularization parameter.
%    epsilon - The error value satisfied by the outer loop iteration.
%    iter - maximum number of iterations.
% OUPUTS:
%    u_update - name of the reconstructed image.
%    t - The time consumed by the algorithm.
%    k - Number of iteration steps of PBCA algorithm.
%    min_w - name of the minimum value of w during iteration.

addpath data
addpath solvers
%% Read Image
%       f = mat2gray(double(imread('fluocells1','tif')));
%       f = mat2gray(double(imread('peppers','png')));
      f = mat2gray(double(rgb2gray(imread('two_code256','png'))));
 
%% Imnoising
[m,n] = size(f);

     H = fspecial('disk',3);
%      H = fspecial('gaussian',7,3);
%      H = fspecial('average',1);

%    eta = 16;
%    f1 = imfilter(f, H);
%    gi = poissrnd(f1*eta)/eta; 
%    g = gi+ 0.1*randn(size(f));


%     load('fluocells1_1_01');
%     load('fluocells1_1_001'); 
%     load('fluocells1_4_01');
%     load('fluocells1_4_001'); 
%     load('fluocells1_16_01'); 
%     load('fluocells1_16_001'); 
%     load('fluocells1_gau_7_3_1_01'); 
%     load('fluocells1_gau_7_3_1_001'); 
%     load('fluocells1_gau_7_3_4_01'); 
%     load('fluocells1_gau_7_3_4_001'); 
%     load('fluocells1_gau_7_3_16_01'); 
%     load('fluocells1_gau_7_3_16_001'); 
%     load('fluocells1_disk_3_1_01'); 
%     load('fluocells1_disk_3_1_001'); 
%     load('fluocells1_disk_3_4_01'); 
%     load('fluocells1_disk_3_4_001'); 
%     load('fluocells1_disk_3_16_01'); 
%     load('fluocells1_disk_3_16_001'); 
 
%     load('peppers_1_01');
%     load('peppers_1_001');
%     load('peppers_4_01');
%     load('peppers_4_001');
%     load('peppers_16_01');
%     load('peppers_16_001');
%     load('peppers_gau_7_3_1_01'); 
%     load('peppers_gau_7_3_1_001');  
%     load('peppers_gau_7_3_4_01');  
%     load('peppers_gau_7_3_4_001'); 
%     load('peppers_gau_7_3_16_01');  
%     load('peppers_gau_7_3_16_001');  
%     load('peppers_disk_3_1_01');  
%     load('peppers_disk_3_1_001');  
%     load('peppers_disk_3_4_01');  
%     load('peppers_disk_3_4_001');  
%     load('peppers_disk_3_16_01');  
%     load('peppers_disk_3_16_001');

%     load('two_code256_1_01');
%     load('two_code256_1_001'); 
%     load('two_code256_4_01');
%     load('two_code256_4_001'); 
%     load('two_code256_16_01'); 
%     load('two_code256_16_001');
%     load('two_code256_gau_7_3_1_01');  
%     load('two_code256_gau_7_3_1_001'); 
%     load('two_code256_gau_7_3_4_01');  
%     load('two_code256_gau_7_3_4_001');  
%     load('two_code256_gau_7_3_16_01');  
%     load('two_code256_gau_7_3_16_001');  
%     load('two_code256_disk_3_1_01');  
%     load('two_code256_disk_3_1_001'); 
%     load('two_code256_disk_3_4_01');  
%     load('two_code256_disk_3_4_001');  
%     load('two_code256_disk_3_16_01');  
    load('two_code256_disk_3_16_001');

    
    PSNRIn = 20*log10(1/sqrt(mean((g(:)-f(:)).^2)));
    SSIMIn = ssimCompt(255*f,255*g);
%% Set Parameters
   epsilon = 1e-4;
   iter = 1000;
   lambda1 = [ 1500 ]; 
   lambda2 = [ 8.4 ]; 
   p1 = 300;
   p2 = 50;
   alpha = 0.003;
   
%% Denoising
       
[u_update,k,SSIM,PSNR,t,min_w]= tvic_TV_mix_denoise_PBCA(f,g,H,iter,epsilon,lambda1,lambda2,p1,p2,alpha);

%% plot the image
figure(1);imshow(f,[0,1]);title('Original');
figure(2);imshow(g,[0,1]);title('Noisy');
figure(3);imshow(u_update,[0,1]);title('Recover');

