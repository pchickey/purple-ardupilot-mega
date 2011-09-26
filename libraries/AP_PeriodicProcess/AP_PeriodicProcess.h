
#ifndef __AP_PERIODICPROCESS_H__
#define __AP_PERIODICPROCESS_H__

class AP_PeriodicProcess
{
    public:
        AP_PeriodicProcess() {};
        virtual void Init() {};
        virtual void register_process(void (* proc)(void));
};

#endif // __AP_PERIODICPROCESS_H__

