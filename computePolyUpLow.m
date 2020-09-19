function VxPoly=computePolyUpLow(boundary)
  
    Vxmin=boundary(1);
    Vxmax=boundary(2);
    
    fy=@(x)1./x;
    fcorn=@(x,y)([2*(x.*y)/(x+y), 2/(x+y)]);
    y1=@(x)(-1/Vxmax^2)*x+2/Vxmax;
    y2=@(x)(-1/Vxmin^2)*x+2/Vxmin;
    
    p1=[Vxmax, fy(Vxmax)] ;
    p2=fcorn(Vxmin, Vxmax);
    p3=[Vxmin, fy(Vxmin)] ;
    
    
     
    VxPoly=[p1(1) p1(2);...  
            p2(1) p2(2);...  
            p3(1) p3(2);...  
            ];  

          
%     VxPoly=[Vxmax 1/Vxmax;...  
%             Vxmin 1/Vxmax;...  
%             Vxmin 1/Vxmin;...  
%             
%             ];     
end

