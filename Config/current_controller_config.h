#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

#define K_D .05f                     // Volts/Amp
#define K_Q .05f                     // Volts/Amp
#define K_SCALE 0.0001f            // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.0255f                  // 1/samples
#define KI_Q 0.0255f                  // 1/samples
#define V_BUS 24.0f                 // Volts

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

#define I_MAX 40.0f



#endif
