//////////////////////////////////////////////////////////////////////////////
// This file is part of 'pcavLib'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'pcavLib', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
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
