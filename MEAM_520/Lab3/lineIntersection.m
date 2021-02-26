function [intersection] = lineIntersection(p11, p12, p21, p22)

N = 100;
tolerance = 5;
intersection = false;

x1 = linspace(p11(1),p12(1),N);
y1 = linspace(p11(2),p12(2),N);
z1 = linspace(p11(3),p12(3),N);

x2 = linspace(p21(1),p22(1),N);
y2 = linspace(p21(2),p22(2),N);
z2 = linspace(p21(3),p22(3),N);

for ii = 1:N
    for jj = 1:N
        if abs(x1(ii)-x2(jj)) < tolerance && abs(y1(ii)-y2(jj)) < ... 
            tolerance && (z1(ii)-z2(jj)) < tolerance
            intersection = true;
            return;
        end
    end
end

end

