clear 
close all 
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 25.13;  % Tiempo para completar el círculo (2*pi*R / velocidad)
ts = 0.1;
t = 0: ts: tf;
N = length(t);

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1); 
y1 = zeros(1,N+1); 
phi = zeros(1,N+1);

% Iniciamos en la raíz (4, 0)
x1(1) = 4; 
y1(1) = 0; 
phi(1) = pi/2;  % Apuntando hacia arriba

hx = zeros(1,N+1); 
hy = zeros(1,N+1);
hx(1) = x1(1); 
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N);

R = 4;           % Radio del círculo
v_lineal = 1;    % Velocidad lineal constante

for k = 1:N
    tk = t(k);
    
    % Velocidad lineal constante
    u(k) = v_lineal;
    
    % Velocidad angular para seguir un círculo: w = v/R
    w(k) = v_lineal / R;
    
    % Si quieres que se detenga después de completar el círculo:
    if tk > (2*pi*R / v_lineal)
        u(k) = 0;
        w(k) = 0;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N
    phi(k+1) = phi(k) + w(k)*ts;
    x1(k+1) = x1(k) + u(k)*cos(phi(k+1))*ts;
    y1(k+1) = y1(k) + u(k)*sin(phi(k+1))*ts;
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
axis([-7 7 -6 5 0 2]);

scale = 4;
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