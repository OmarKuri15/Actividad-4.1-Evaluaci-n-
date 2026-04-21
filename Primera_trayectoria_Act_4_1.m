clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 5.18;             % Tiempo final para X [0 a 5]
ts = 0.01;          % Paso de tiempo fino
t = 0: ts: tf;
N = length(t);

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);
y1 = zeros(1,N+1);
phi = zeros(1,N+1);

x1(1) = 0;
y1(1) = 0;          % f(0) = 2*sin(0) = 0
phi(1) = 0;         % Orientación inicial

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);
hy = zeros(1,N+1);
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N);
w = zeros(1,N);

% La derivada se usa porque da la pendiente de la trayectoria, o sea, la dirección del robot.
% La segunda derivada dice qué tanto cambia esa pendiente, que es básicamente la curvatura.
% Y con eso se calcula la velocidad angular, porque el robot tiene que girar más cuando la curva es más cerrada.

for k = 1:N
    tk = t(k);
    
    % x avanza con el tiempo
    x_val = tk;
    
    % Derivada dy/dx = 4*x*cos(x^2)
    dydx = 4 * x_val * cos(x_val^2);
    
    % dx/dt = 1
    dx = 1;
    dy = dydx * dx;
    
    % Velocidad lineal
    u(k) = sqrt(dx^2 + dy^2);
    
    % Segunda derivada d²y/dx² = 4*cos(x^2) - 8*x²*sin(x^2)
    d2ydx2 = 4 * cos(x_val^2) - 8 * (x_val^2) * sin(x_val^2);
    
    % Curvatura
    kappa = d2ydx2 / (1 + dydx^2)^(3/2);
    
    % Velocidad angular
    w(k) = u(k) * kappa;
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N
    tk = t(k);
    x_val = tk;
    
    % Calcular posición directamente de la función (sin acumular errores)
    x1(k+1) = x_val;
    y1(k+1) = 2 * sin(x_val^2);
    
    % Calcular orientación basada en la derivada
    dydx = 4 * x_val * cos(x_val^2);
    phi(k+1) = atan2(dydx, 1);
    
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene=figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

view([0 90]);
axis([-1 7 -3 3 0 2]);

scale = 3;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

step=1;

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;
set(graph,'position',sizeScreen);

subplot(211);
plot(t, u, 'b', 'LineWidth', 2);
grid on;
ylabel('m/s');
xlabel('Tiempo [s]');
legend('u');

subplot(212);
plot(t, w, 'r', 'LineWidth', 2);
grid on;
ylabel('[rad/s]');
xlabel('Tiempo [s]');
legend('w');