function x=euler_alg(dx_dt,xini,h,tmax)

    x(1)=xini;
    for i=1:tmax-1,
        x(i+1)=x(i)+dx_dt(x(i))*h;
    end
    