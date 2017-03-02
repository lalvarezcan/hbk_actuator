#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

#define K_D .5f   //V/A
#define K_Q .5f   //V/A
#define KI_D 0.04f  //1/samples
#define KI_Q 0.04f  //1/samples
#define V_BUS 14.0f

#define D_INT_LIM V_BUS/(K_D*KI_D)  //A*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  //A*samples



#endif
