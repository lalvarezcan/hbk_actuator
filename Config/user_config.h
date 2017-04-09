/// Values stored in flash, which are modifieable by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define E_OFFSET                __float_reg[0]                                  // Encoder electrical offset
#define M_OFFSET                __float_reg[1]                                  // Encoder mechanical offset
#define I_BW                    __float_reg[2]                                  // Current loop bandwidth
#define TORQUE_LIMIT            __float_reg[3]                                  // Torque limit (current limit = torque_limit/(kt*gear ratio))

#define PHASE_ORDER             __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                  __int_reg[1]                                    // CAN bus ID
#define ENCODER_LUT             __int_reg[4]                                    // Encoder offset LUT - 128 elements long

extern float __float_reg[];
extern int __int_reg[];

#endif
