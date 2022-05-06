function x = proj_bound(xi,L,U)
xw = max(xi,L);
x = min(xw,U);
end