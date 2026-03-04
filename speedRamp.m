function [y, f1, f2, f3] = speedRamp(X)
    syms x c1 c2 c3 a real
    h3 = 3.*x.^2-2.*x.^3;
    
    segment1 = a*subs(h3, x, (x-c1)/(c2-c1));
    segment2 = a;
    segment3 = a*(1-subs(h3, x, (x-c3)/(30-c3)));
    
    vals = [c1 c2 c3 a; 4 10 25.2 2.1];
    
    f1 = subs(segment1, vals(1, :), vals(2, :));
    f2 = subs(segment2, vals(1, :), vals(2, :));
    f3 = subs(segment3, vals(1, :), vals(2, :));

    C1 = 4; C2 = 10; C3 = 25.2;
    if X <= C1
        y = 0;
    elseif X <= C2
        y = subs(f1, x, X);
    elseif X <= C3
         y = subs(f2, x, X);
    elseif X <= 30
        y = subs(f3, x, X);
    else
        y = 0;
    end
end