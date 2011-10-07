
#ifndef __AP_PERIODICPROCESS_H__
#define __AP_PERIODICPROCESS_H__

class AP_PeriodicProcess
{
    public:
        AP_PeriodicProcess() {};
        virtual void register_process(void (* proc)(void)) = 0;
};

#endif // __AP_PERIODICPROCESS_H__

