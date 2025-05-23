clear all;
clc;
% Medidas de los eslabones.
a1 = 1.32;  
a2 = 6.97; 
a3 = 7.97;
% Ángulos de las articulaciones.
q1 = 90;
q2 = 45;
matriz1 = [0, 0, 1, 0;
           0, 1, 0, 0;
           -1, 0, 0, a1;
           0, 0, 0, 1];
matriz2 = [cosd(q1), -sind(q1), 0, 0;
           sind(q1), cosd(q1), 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
matriz3 = [1, 0, 0, 0;
           0, 1, 0, a2;
           0, 0, 1, 0;
           0, 0, 0, 1];
matriz4 = [cosd(q2), -sind(q2), 0, 0;
           sind(q2), cosd(q2), 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
matriz5 = [1, 0, 0, 0;
           0, 0, 1, a3;
           0, -1, 0, 0;
           0, 0, 0, 1];
resultado = matriz1 * matriz2 * matriz3 * matriz4 * matriz5;
% Resultado.
disp('Matriz de transformación homogénea:');
disp(resultado);
% Cinemática directa.
x = (a2 * cosd(q1) + a3 * cosd(q1 + q2));
y = a1 + a2 * sind(q1) + a3 * sind(q1 + q2);
disp(['Posición del extremo del robot (x, y): (', num2str(x), ', ', num2str(y), ')']);
D = y - a1;
cos_q2 = (x^2 + D^2 - a2^2 - a3^2) / (2 * a2 * a3);
if cos_q2 >= -1 && cos_q2 <= 1
    sin_q2 = sqrt(1 - cos_q2^2);          
    q2 = atan2d(sin_q2, cos_q2)% ángulo q2 en grados
    q1 = atan2d(D, x) - atan2d(a3 * sin_q2, a2 + a3 * cos_q2)% ángulo q1
else
    disp('No hay solución real para q2 (posición fuera del alcance del robot).');
end
