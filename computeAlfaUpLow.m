function alphas=computeAlfaUpLow(Vx, boundary)

    lambda=0;
    alphas=zeros(3,1);
    
    Vxmin=boundary(1);
    Vxmax=boundary(2);
    
    fy=@(x)1./x;
    fcorn=@(x,y)([2*(x.*y)/(x+y), 2/(x+y)]);
    y1=@(x)(-1/Vxmax^2)*x+2/Vxmax;
    y2=@(x)(-1/Vxmin^2)*x+2/Vxmin;
    
    p1=[Vxmax, fy(Vxmax)] ;
    p2=fcorn(Vxmin, Vxmax);
    p3=[Vxmin, fy(Vxmin)] ;
    
%     p1=[Vxmax, fy(Vxmax)] ;
%     p3=[Vxmin, fy(Vxmin)] ;
%     p2=[Vxmin, 1/Vxmax];
    
    vA=[p1(1) p2(1) p3(1);...
    p1(2) p2(2) p3(2);...
    1 1 1];

    vb=[Vx;1/Vx;1];

    regvA=vA'*vA+lambda*(eye(size(vA'*vA)));
    regyb=vA'*vb;

    alphas=regvA\regyb;
    alphas(alphas<0)=0;
 
end

