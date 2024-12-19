% Define the lengths of the links
l1 = 2.7;  % Length of link AB
l2 = 2.5;  % Length of link AE
l3 = 10;  % Length of link ED
l4 = 2.7;  % Length of link DC
l5 = 10;  % Length of link CB
l6=10.3;

%User input
X = input('Enter X coordinate of end-effector: ');
Y = input('Enter Y coordinate of end-effector: ');

% Calculate k1
k1 = (X^2 + l5^2 + l1^2 + Y^2 - l6^2 - 2 * X * l1) / (2 * l5);


% Define joint angles and vertices
theta5 = 2 * atan2((Y +sqrt((X - l1)^2 + Y^2 - k1^2)), (k1 + (X - l1)));
theta5_deg=theta5*(180 / pi)
theta6 = atan2((X - l5 * cos(theta5) - l1), (l5 * sin(theta5) - Y));
theta6_deg=theta6*(180 / pi)

x2 = X - (l4 + l6) * sin(theta6)  % Calculation for x2 
y2 = Y + (l4 + l6) * cos(theta6)   % Calculation for y2 

% Calculate k2
k2 = (x2^2 + y2^2 + l2^2 - l3^2) / (2 * l2);

theta1 = 2 * atan2((y2 -sqrt(x2^2 + y2^2 - k2^2)),(k2 - x2));
theta2 = atan2((y2 + l2 * sin(theta1)),(x2 + l2 * cos(theta1)));

theta1_deg=theta1*(180 / pi)
theta2_deg=theta2*(180 / pi)


%All other angles can be defined in terms of theta1 and theta 5



% Define position variables (x's and y's)
x1 = -l2 * cos(theta1);  % Calculation for x1
y1 = l2 * sin(theta1);    % Calculation for y1

x3 = l5 * cos(theta5) + l1; % Calculation for x3
y3 = l5 * sin(theta5);       % Calculation for y3

x4 = l1; 
y4 = 0; 

x5 = 0; 
y5 = 0; 



% Define the coordinates of the points
A = [x5, y5];                % Point A at the origin
B = [x4, y4];               % Point B (at length l1 from A)
E = [x1, y1];                % Arbitrary coordinates for E
D = [x2, y2];                % Arbitrary coordinates for D
C = [x3, y3];                % Arbitrary coordinates for C




%calculate Forward Kinematics based on the angles generated
X_real = l5 * cos(theta5) + l1 + l6 * sin(theta6)  % Calculation for X
Y_real = l5 * sin(theta5) - l6 * cos(theta6)      % Calculation for Y

% Create the figure
figure;
hold on;

% Plot the links
plot([A(1), B(1)], [A(2), B(2)], 'b', 'LineWidth', 2); % Link AB
plot([A(1), E(1)], [A(2), E(2)], 'g', 'LineWidth', 2); % Link AE
plot([E(1), D(1)], [E(2), D(2)], 'r', 'LineWidth', 2); % Link ED
plot([D(1), C(1)], [D(2), C(2)], 'm', 'LineWidth', 2); % Link DC
plot([C(1), B(1)], [C(2), B(2)], 'c', 'LineWidth', 2);       % Link CB
plot([C(1),X], [C(2),Y], 'y', 'LineWidth', 2);       % Link CB

% Plot points A, B, C, D, E
plot(A(1), A(2), 'ko', 'MarkerFaceColor', 'k'); % Point A
text(A(1), A(2), ' A', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

plot(B(1), B(2), 'ko', 'MarkerFaceColor', 'b'); % Point B
text(B(1), B(2), ' B', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

plot(C(1), C(2), 'ko', 'MarkerFaceColor', 'c'); % Point C
text(C(1), C(2), ' C', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

plot(D(1), D(2), 'ko', 'MarkerFaceColor', 'm'); % Point D
text(D(1), D(2), ' D', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

plot(E(1), E(2), 'ko', 'MarkerFaceColor', 'g'); % Point E
text(E(1), E(2), ' E', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Plot the end effector
plot(X_real, Y_real, 'ko', 'MarkerFaceColor', 'y'); % End-effector
text(X_real, Y_real, ' End-Effector', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Set axis limits
xlim([-10, 20]);
ylim([-10, 20]);
grid on;
title('Quadruped Leg Visualization');
xlabel('X-axis');
ylabel('Y-axis');
hold off;
