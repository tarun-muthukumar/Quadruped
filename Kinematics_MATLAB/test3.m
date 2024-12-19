% Define the lengths of the links
lab = 2.7;  % Length of link AB
lae = 2.5;  % Length of link AE
led = 10;  % Length of link ED
ldc = 2.7;  % Length of link DC
lcb = 10;  % Length of link CB
lcf = 10.3; %Length of link CF

%Section formula Ratios
m = ldc; 
n = lcf;

%Thetas from IK
theta1 =  -5.174807128222946
theta4 = 0.945383081119290

rad2deg(theta1)
rad2deg(theta4)

%Coordinates of Point C
c_x = lcb*cos(theta4) + lab;
c_y = lcb*sin(theta4);

%Coordinates of Point E
e_x = -lae*cos(theta1);
e_y = lae*sin(theta1);

%Solving for Coordinates of D using Distance formula.
%Length ED is known and Length CD is known, so now D can be determined

syms x y

eq1 = 2*(x^2) + 2*(y^2) - (x*2*(e_x + c_x)) - (2*y*(e_y + c_y)) == (led^2 + ldc^2) - e_x^2 - c_x^2 - e_y^2 - c_y^2; 
eq2 = x*(2*c_x - 2*e_x) + y*(2*c_y - 2*e_y) == (led^2 - ldc^2) - e_x^2 + c_x^2 - e_y^2 + c_y^2;

solutions = solve([eq1, eq2], [x, y]);

%Multiple Solution arises here
xSol = double(solutions.x);
ySol = double(solutions.y);

X = ((c_x*(m+ n) - n*xSol) / m);
Y = ((c_y*(m+n) - n*ySol) / m);
X = X(2)
Y = Y(2)


