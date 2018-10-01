close all;
%% Generacion de la señal aleatoria para la identificacion
figure(1)
plot(datos.Time,datos.Data(:,1));
figure(2)
plot(datos.Time,datos.Data(:,2),'r');

y=datos.Data(:,2);
u=datos.Data(:,1);
TimeVector=datos.Time;


data = iddata(y,u,[],'SamplingInstants',TimeVector)

	na=0;
	nb=2;
	nk=1;

sistema_estimado = arx(data,[na nb nk])

[Numerador Denominador] = polydata(sistema_estimado);
sys = tf(Numerador,Denominador)

	