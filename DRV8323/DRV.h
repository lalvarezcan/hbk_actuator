#ifndef DRV_H
#define DRV_H

/// Registers ///
#define FSR1            0x0     ///Fault Status Register 1///
#define FSR2            0x1     ///Fault Status Register 2///
#define DCR             0x2     ///Drive Control Register///
#define HSR             0x3     ///Gate Drive HS Register///
#define LSR             0x4     ///Gate Drive LS Register///
#define OCPCR           0x5     ///OCP Control Register///
#define CSACR           0x6     ///CSA Control Register///

#define PWM_MODE_6X     0x0     ///PWM Input Modes///
#define PWM_MODE_3X     0x1
#define PWM_MODE_1X     0x2
#define PWM_MODE_IND    0x3
#define CSA_GAIN_5      0x0     ///Current Sensor Gain///
#define CSA_GAIN_10     0x1
#define CSA_GAIN_20     0x2
#define CSA_GAIN_40     0x3
#define DIS_SEN_EN      0x0     ///Overcurrent Fault
#define DIS_SEN_DIS     0x1

class DRV832x {
    public:
        DRV832x(SPI *spi, DigitalOut *cs);
        int read_FSR1();
        int read_FSR2();
        int read_register(int reg);
        void write_register(int reg, int val);
        void write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT);
        void write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS);
        void write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS);
        void write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL);
        void write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL);
        void enable_gd(void);
        void disable_gd(void);
        void calibrate(void);
        void print_faults();
        
    private:
        SPI *_spi;
        DigitalOut *_cs;
        uint16_t spi_write(uint16_t val);
};

#endif