#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

#define K_D .05f                     // Volts/Amp
#define K_Q .05f                     // Volts/Amp
#define KI_D 0.04f                  // 1/samples
#define KI_Q 0.04f                  // 1/samples
#define V_BUS 24.0f                 // Volts

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples



#endif
