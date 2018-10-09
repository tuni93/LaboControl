function [sys,x0,str,ts]=myPID(t,x,u,flag,Kp,Kd,Ki,N,Ts,lowU,highU)
%% Valores por defecto
if nargin==9,
    lowU=-1E20;highU=1E20; %Saturaciones por defecto
end

    %%
% Kp=constante proporcional
% Kd=constante derivativa
% Ki=constante integrativa
% N =Orden del filtro derivativo
% Ts =tiempo de muestreo

%SFUNTMPL General M-file S-function template
%   With M-file S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an M-File S-function syntax is:
%       [SYS,X0,STR,TS] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].

%   Copyright 1990-2002 The MathWorks, Inc.
%   $Revision: 1.18.2.1 $

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(Ts);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,Kp,Kd,Ki,N,Ts,lowU,highU);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,Kp,Kd,Ki,N,Ts,lowU,highU);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u,Ts);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(Ts)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 2;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [0 0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [Ts 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,Kp,Kd,Ki,N,Ts,lowU,highU)
%Recibe
%   x(1)=Ik    x(2)=D'k    u(1)=rk      u(2)=yk
%Devuelve
%   x(1)=Ik+1     x(2)=D'k+1
    gamma=Kd/N;
        
    %
    %D_k=gamma/gamma+h·D_{k-1} - KpKd/gamma+h (yk-y_{k-1})
    %D_k=gamma/gamma+h·D_{k-1} + KpKd/gamma+h·y_{k-1} - KpKd/gamma+h·yk
    %
    % Separo
    % D_k       =  D'_k + D''_k
    % D'_k      =  gamma/gamma+h·D_{k-1} +KpKd/gamma+h·y_{k-1}
    % D'_k+1    =  gamma/gamma+h·D_k +KpKd/gamma+h·y_k
    % D''_k     = -KpKd/gamma+h·yk
    %
    % Dk:
        Dpp_k = -Kp*Kd/(gamma+Ts)*u(2);
    Dk = x(2)+Dpp_k;
    % D'k+1:
    sys(2)=gamma/(gamma+Ts) *Dk + Kp*Kd/(gamma+Ts)*u(2) ;

    %
    %  Ik=KpKiTs+\sum_{j=0}^{k-1} e_j
    %· Ik+1=Ik + KpKiTs e_k
    %
    sys(1)=x(1)+Kp*Ki*Ts*(u(1)-u(2));
    
    %%AntiWindUp
    % Resuelto sin variables globales
    %
    % Si satura, que no actualice Ik.
    % Para ello tengo que saber la salida del pid
    Pk   = Kp*(u(1)-u(2));
    Ik   = x(1);
    Dk   = x(2)-Kp*Kd/(gamma+Ts)*u(2);
    yk  = Ik + Dk + Pk ;
    if yk>=highU || yk<=lowU,  %si satura,
        sys(1)=x(1);            % que no acumule error
    end
    
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,Kp,Kd,Ki,N,Ts,lowU,highU)
%Recive
%   x(1)=I_k    x(2)=D'_k    u(1)=rk      u(2)=yk

   gamma=Kd/N; 
   % factor proporcional
    Pk   = Kp*(u(1)-u(2));
    
   %calculo Ik
    Ik   = x(1); 
    
   % Calculo factor derivativo
   % D_k  =  D'_k + D''_k
   %
     Dpp_k = -Kp*Kd/(gamma+Ts)*u(2);
    Dk   = x(2) + Dpp_k;

   % Emito la salida
    sys  = Ik + Dk + Pk ;
    
    if sys>=highU, % Si satura, que no saque la salida
        sys=highU
    elseif sys<=lowU,
        sys=lowU
    end

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u,Ts)

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

        