%% genero las muestras a estimar.
	TimeVector=datos.Time;
	u=datos.Data(:,1);
	y=datos.Data(:,2);
	data = iddata(y,u,[],'SamplingInstants',TimeVector)
