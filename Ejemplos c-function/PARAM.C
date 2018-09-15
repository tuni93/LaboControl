#define S_FUNCTION_NAME  param
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);       /* cantidad de parametros */

    ssSetNumInputPorts(S, 1);       /* numero de puertos de entrada */
    ssSetInputPortWidth(S, 0, 1);   /* ancho del primer u unico puerto de entrada */
    ssSetInputPortDirectFeedThrough(S, 0, 1);   /* el puerto es de paso directo */

    ssSetNumOutputPorts(S,1);       /* numero de puertos de salida */
    ssSetOutputPortWidth(S, 0, 1);  /* ancho del puerto de salida */
    ssSetNumSampleTimes(S, 1);      /* un sólo tiempo de muestreo */
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);   /* tiempo heredado */
    ssSetOffsetTime(S, 0, 0.0);                     /* offset 0 */
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *p;   /* declaro puntero */
    real_T            *y = ssGetOutputPortRealSignal(S,0);    /* puntero a la salida */
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); /* puntero a direcc. de entrada */
    p = mxGetPr(ssGetSFcnParam(S,0));   /* puntero a param. y luego obtiene el valor del param. */
    *y = *p *(*uPtrs[0]);               /* salida = paramerto * entrada */
   
}


static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    
#include "simulink.c"      
#else
#include "cg_sfun.h"    
#endif
