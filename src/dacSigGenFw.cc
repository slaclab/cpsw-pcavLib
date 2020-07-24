#include "dacSigGenFw.h"

#include <cpsw_yaml.h>
#include <yaml-cpp/yaml.h>
#include <cpsw_sval.h>
#include <cpsw_entry_adapt.h>
#include <cpsw_hub.h>
#include <fstream>
#include <sstream>

#include <math.h>


#define CPSW_TRY_CATCH(X)       try {   \
        (X);                            \
    } catch (CPSWError &e) {            \
        fprintf(stderr,                 \
                "CPSW Error: %s at %s, line %d\n",     \
                e.getInfo().c_str(),    \
                __FILE__, __LINE__);    \
        throw e;                        \
    }

class CdacSigGenFwAdapt;
typedef shared_ptr<CdacSigGenFwAdapt> dacSigGenFwAdapt;

class CdacSigGenFwAdapt : public IdacSigGenFw, public IEntryAdapt {
private:
    Path      _pDacSigGen;
    ScalVal   enableMask_;
    ScalVal   modeMask_;
    ScalVal   signFormat_;
    ScalVal   periodSize_;
    ScalVal   i_waveform_;
    ScalVal   q_waveform_;

protected:
    int16_t  i_wf_out[MAX_SAMPLES];
    int16_t  q_wf_out[MAX_SAMPLES];

public:
    CdacSigGenFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie);

    virtual void  setIWaveform(double *i_waveform);
    virtual void  setQWaveform(double *q_waveform);

};


dacSigGenFw IdacSigGenFw::create(Path p)
{
    return IEntryAdapt::check_interface<dacSigGenFwAdapt, DevImpl>(p);
}

CdacSigGenFwAdapt::CdacSigGenFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie):
    IEntryAdapt(k, p, ie),
    _pDacSigGen(p->findByName("mmio/AppTop/DacSigGen")),
    enableMask_(IScalVal::create(_pDacSigGen->findByName("EnableMask"))),
    modeMask_(  IScalVal::create(_pDacSigGen->findByName("ModeMask"))),
    signFormat_(IScalVal::create(_pDacSigGen->findByName("SignFormat"))),
    periodSize_(IScalVal::create(_pDacSigGen->findByName("PeriodSize"))),
    i_waveform_(IScalVal::create(_pDacSigGen->findByName("Waveform[0]/MemoryArray"))),
    q_waveform_(IScalVal::create(_pDacSigGen->findByName("Waveform[1]/MemoryArray")))

{
    uint8_t v;
    v = 0x03; CPSW_TRY_CATCH(enableMask_->setVal(v));    // enable two waveforms I and Q
    v = 0x00; CPSW_TRY_CATCH(modeMask_->setVal(v));      // triggered mode
    v = 0x00; CPSW_TRY_CATCH(signFormat_->setVal(v));    // signed 2's complementary data type
    CPSW_TRY_CATCH(periodSize_->setVal(MAX_SAMPLES));    // length of IQ table
    
}


void CdacSigGenFwAdapt::setIWaveform(double *i_waveform)
{
    for(int i = 0; i < MAX_SAMPLES; i++) {
        i_wf_out[i] =(int16_t)(*(i_waveform +i) * 0x7fff);
    }

    CPSW_TRY_CATCH(i_waveform_->setVal((uint16_t *)i_wf_out,MAX_SAMPLES));
}

void CdacSigGenFwAdapt::setQWaveform(double *q_waveform)
{
    for(int i = 0; i < MAX_SAMPLES; i++) {
        q_wf_out[i] =(int16_t)(*(q_waveform +i) * 0x7fff);
    }

    CPSW_TRY_CATCH(i_waveform_->setVal((uint16_t *)q_wf_out,MAX_SAMPLES));
}
