/// Values stored in flash, which are modifieable by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define E_OFFSET                __float_reg[0]
#define M_OFFSET                __float_reg[1]
#define I_BW                    __float_reg[2]
#define I_LIMIT                 __float_reg[3]

#define PHASE_ORDER             __int_reg[0]
#define CAN_ID                  __int_reg[1]
#define ENCODER_LUT             __int_reg[4]

extern float __float_reg[];
extern int __int_reg[];

#endif
