function [sys,x0,str,ts] = velocidadEncoder(t,x,u,flag,encoder_ppv,Ts)

switch flag,

  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(t,x,u,flag);

  case 1,
    sys=mdlDerivatives(t,x,u,flag);%Est continuos

  
  case 2,
    sys=mdlUpdate(t,x,u,flag,encoder_ppv,Ts);%Ecuacion en diferencias

  case 3,
    sys=mdlOutputs(t,x,u,flag,encoder_ppv,Ts);

 
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u,encoder_ppv,Ts);

  case 9,
    sys=mdlTerminate(t,x,u);


  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u,flag)


sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0 0 0];


str = [];

ts  = [0 0];

function sys=mdlDerivatives(t,x,u,flag)
sys = [];

function sys=mdlUpdate(t,x,u,flag,encoder_ppv,Ts)
%u(1)= Va Sensor A
%u(2)= Vb Sensor B
%sys(2)=Contador hasta que cambie de estado.
%sys(3)=Maximo valor de la cuenta de sys(2).

	estado=u(1)+10*u(2); % Esto es para hacer un representacion de maquina de estados 00 01 10 11
	if(estado~= x(1)) 
		sys(3)=x(2);
		sys(2)=1;
	else
		sys(2)=x(2)+1;
		sys(3)=x(3);
	end
	sys(1)=estado;
	


% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,flag,encoder_ppv,Ts)
	numero_vueltas=encoder_ppv;
	if(t<=0 || x(3)==0)
		sys(1)=0;
	else
		sys(1)=pi()/(x(3)*Ts*(2^numero_vueltas));
		end
%end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u,encoder_ppv,Ts)

sampleTime = Ts;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate