% Función para mostrar telemetría de robot móvil
% Laboratorio de Robótica 4º GIERM 18-19
% Federico Cuesta
% USO: Pasar como parámetro el nombre del archivo de telemetría con el
% sigiente contenido por fila:
% -Incremento de tiempo transcurrido desde la lectura anterior (milisegundos),  
% -Distancia medida por sensor Izq/Frontal (cm), 
% -Distancia medida por sensor Dch/Trasero(cm), 
% -Referencia de control (cm), 
% -Modo activo (0: Parado, 1: Control frontal, … 4),
% -Velocidad PWM motor Izq (+/-255, negativo indica marcha atrás), 
% -Velocidad PWM motor Dch (+/-255, negativo indica marcha atrás).

function telemetria(archivo)

tel=load(archivo);
muestras=length(tel);
disp('Incremento de tiempo mínimo:'); disp(min(tel(:,1)));
disp('Incremento de tiempo máximo:');disp(max(tel(:,1)));
disp('Incremento de tiempo promedio:'); disp(mean(tel(:,1)));
tiempo=zeros(1,muestras);
tiempo(1)=tel(1,1); %Vector de tiempo absoluto
for i=2:muestras
    tiempo(i)=tiempo(i-1)+tel(i,1);
end    


subplot(2,1,1);
plot(tiempo,tel(:,2), tiempo,tel(:,3), tiempo,tel(:,4));
xlabel('Tiempo (ms)');
title('Sensores');

subplot(2,1,2);
plot(tiempo,tel(:,5), tiempo,tel(:,6), tiempo,tel(:,7));
title('Actuadores');
xlabel('Tiempo (ms)');

end