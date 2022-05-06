
======================================================================
REFERENCE:
======================================================================   
Miao Chen and Yuchao Tang and Jie Zhang and Tieyong Zeng; Proximal ADMM approach for image restoration with
mixed Poisson–Gaussian noise.

======================================================================
SOFTWARE
======================================================================   

SOFTWARE REVISION DATE:

       May 2022

SOFTWARE LANGUAGE:

       MATLAB R2020b

======================================================================

The directory contains the following files

README         	                     : This file
demo_tvic_Huber_mix_denoise_PBCA.m   : Example of how to use PBCA+Huber algorithm deblurring method
demo_tvic_TV_mix_denoise_PBCA.m      : Example of how to use PBCA+TV algorithm deblurring method


--------------------------------------------------------
data                            : This folder contains three test images and corresponding noise images.

--------------------------------------------------------
solvers	                        : This folder contains the following functions.
tvic_Huber_mix_denoise_PBCA.m   : function implementing the PBCA+Huber deblurring method
tvic_TV_mix_denoise_PBCA.m      : function implementing the PBCA+TV deblurring method
prox_gamma_huber.m              : function implementing the Huber proximity operator
proj_bound.m	                : function of projection operator
ssimCompt.m                     : function of SSIM
D.m                             : Forward finite difference operator
Dt.m		                    : Transpose of the forward finite difference operator


======================================================================

If you have any questions, feel free to email at yctang@ncu.edu.cn.
