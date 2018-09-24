clear all
clc
close all
%% Generacion de la señal aleatoria para la identificacion
N=100;%Nro de puntos
Ts=0.001;
T_total=(N-1)*Ts;
signal(:,2)=idinput(N,'prbs');
aux=0:Ts:T_total;
aux=aux';
signal(:,1)=aux;
plot(signal(:,1),signal(:,2))