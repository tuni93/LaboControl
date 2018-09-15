function x=funcion_estados(x1,x2)
R = 100;
L = 1;
C = 1;
V = 1;

x1_p=x2/C;
x2_p=-x1/L-R/L*x2+V;

x=[x1_p;x2_p];
end