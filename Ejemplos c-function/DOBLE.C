
#define S_FUNCTION_NAME  doble
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumInputPorts(S, 2);                   /* Un puerto de entrada */
    ssSetInputPortWidth(S, 0, 1);               /* Ancho del único puerto de entrada es 1 */
    ssSetInputPortDirectFeedThrough(S, 0, 1);   /* La función es de paso diercto */
    ssSetInputPortWidth(S, 1, 1);               /* Ancho del único puerto de entrada es 1 */
    ssSetInputPortDirectFeedThrough(S, 1, 1);   /* La función es de paso diercto */

    ssSetNumOutputPorts(S,2);                   /* Un puerto de salida */
    ssSetOutputPortWidth(S, 0, 1);              /* Ancho del único puerto de salida es 1 */
    ssSetOutputPortWidth(S, 1, 1);              /* Ancho del único puerto de salida es 1 */

    ssSetNumSampleTimes(S, 1);                  /* tiene un sólo tiempo de muestreo */
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);   /* ünico tiempo de muestreo es heredado */
    ssSetOffsetTime(S, 0, 0.0);                     /* Offset NULO */
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y0 = ssGetOutputPortRealSignal(S,0);       /* puntero a salida */
    real_T            *y1 = ssGetOutputPortRealSignal(S,1);       /* puntero a salida */	

    InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0); /* puntero a entrada */ 
    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1); /* puntero a entrada */ 	

    *y0 = *uPtrs0[0] + *uPtrs1[0];         /* actualiza salida. (salida = 2 x entrada) */
    *y1 = *uPtrs0[0] * *uPtrs1[0];         /* actualiza salida. (salida = 2 x entrada) */

}


static void mdlTerminate(SimStruct *S)
{
    /* no hay código de finalización */
}

#ifdef  MATLAB_MEX_FILE    
#include "simulink.c"   /* caraga este archivo si es compilado como MEX-FILE */      
#else
#include "cg_sfun.h"    /* caraga este archivo si no es compilada como MEX-FILE (x ej RTW) */
#endif
