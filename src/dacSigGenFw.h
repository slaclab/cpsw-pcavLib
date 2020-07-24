#ifndef _DACSIGGEN_H
#define _DACSIGGEN_H


#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>


#define MAX_SAMPLES  4096

class IdacSigGenFw;
typedef shared_ptr<IdacSigGenFw> dacSigGenFw;

class IdacSigGenFw : public virtual IEntry {
public:
    static dacSigGenFw create(Path p);

    virtual void setIWaveform(double *i_waveform) = 0;
    virtual void setQWaveform(double *q_waveform) = 0;
};



#endif  /* _DACSIGGEN_H */
