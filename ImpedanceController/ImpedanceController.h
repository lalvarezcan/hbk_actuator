


class ImpedanceController{
    public:
        //CurrentRegulator();
        virtual void SetImpedance(float K, float B, float );
    
    private:
        float reference, _K, _B, output;   //Referene position (rad), motor stiffness (N-m/rad), motor damping (N-m*s/rad), output(N-m)
    
    };