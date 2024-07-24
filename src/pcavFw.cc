//////////////////////////////////////////////////////////////////////////////
// This file is part of 'pcavLib'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'pcavLib', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include "pcavFw.h"

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


#define _FIXED18_16_PHASE(P) \
(((P) & 0x20000) ? ((double)(P) - (double)(0x40000)):(double)(P));

#define _FIX_32_18(V) \
(((V) & (1UL<<31))?((double)(V) - (double)(1UL<<32))/(double)(1<<18):(double)(V)/(double)(1<<18))

#define _FIX_18_18(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x40000):(double)(V)/(double)(0x40000))

#define _FIX_18_17(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x20000):(double)(V)/(double)(0x20000))

#define _FIX_18_16(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x10000):(double)(V)/(double)(0x10000))

#define _FIX_18_15(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x8000):(double)(V)/(double)(0x8000))

#define _FIX_2_1(V) \
(((V) & 0x2) ? ((double)(V) - (double)(0x4))/(double)(0x2):(double)(V)/(double)(0x2))

inline static uint32_t nco(double v)
{
    int32_t out = (int32_t) ((v / 1.7E+7) * (double)((uint64_t)0x1<<32));
    
    return (uint32_t) (out >> 6);
    
}

class CpcavFwAdapt;
typedef shared_ptr<CpcavFwAdapt> pcavFwAdapt;

class CpcavFwAdapt : public IpcavFw, public IEntryAdapt {
private:
protected:
    Path pPcavReg_;      // pcav register path
    Path pDiagBus_;      // pcav register path

    ScalVal_RO    version_;      // pcav firmware version

    /* rf reference */
    ScalVal_RO    rfRefAmpl_;    // RF reference amplitude,  fixed point 18.17
    ScalVal_RO    rfRefPhase_;   // RF reference phase, fixed point 18.17
    ScalVal_RO    rfRefI_;       // RF reference I, fixed point 18.17
    ScalVal_RO    rfRefQ_;       // RF reference Q, fixed point 18.17
    ScalVal       rfRefSel_;     // RF reference selection, unsigned fixed 4.0

    ScalVal       wfDataSel_[8];    // wfDataSelector

    
    /* cavity 1 */
    /* probe 1 */

    ScalVal_RO    cav1P1IfAmpl_;    // IF amplitude for cavity 1 probe 1, fixed 18.17
    ScalVal_RO    cav1P1IfPhase_;   // IF phase for cavity 1 probe 1, fixed 18.17
    ScalVal_RO    cav1P1IfI_;       // IF I for cavity 1 probe 1, fixed 18.17
    ScalVal_RO    cav1P1IfQ_;       // IF Q for cavity 1 probe 1, fixed 18.17
    ScalVal       cav1P1ChanSel_;   // Channel Select for cavity 1 probe 1, unsigned fixed 4.0
    ScalVal_RO    cav1P1DCReal_;    // DC real part for cavity 1, probe 1, fixed 21.19
    ScalVal_RO    cav1P1DCImage_;   // DC imaginary part for cavity 1, probe 1, fixed 21.19
    ScalVal       cav1P1WindowStart_;    // Integration window start for cavity 1, probe 1, unsigned fixed 16.0
    ScalVal       cav1P1WindowStop_;     // Integration window stop for cavity 1, probe 1,  unsigned fixed 16.0
    ScalVal_RO    cav1P1DCFreq_;    // DC frequency for cavity 1, probe 1, fixed 26.0
    ScalVal_RO    cav1P1IntegI_;    // Integrated I for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1IntegQ_;    // Integrated Q for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1OutPhase_;     // Output Phase for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1OutAmpl_;      // Output amplitude for cavity 1, probe 1, fixed 18.17

    ScalVal_RO    cav1P1CompPhase_;     // Comparison phase for cavity 1, probe 1, fixed 18.17

    ScalVal       cav1P1CalibCoeff_;    // calibration coefficient (TBD)
    ScalVal       cav1P1PhaseOffset_;   // Phase Offset (TBD)
    ScalVal       cav1P1Weight_;        // Weights (TBD)
    // NCO
    ScalVal       cav1NCOPhaseAdj_;    // NCO phase adjust for cavity 1, unsigned fixed 29.29

    /* probe 2 */

    ScalVal_RO    cav1P2IfAmpl_;    // IF amplitude for cavity 1 probe 2, fixed 18.17
    ScalVal_RO    cav1P2IfPhase_;   // IF phase for cavity 1 probe 2, fixed 18.17
    ScalVal_RO    cav1P2IfI_;       // IF I for cavity 1 probe 2, fixed 18.17
    ScalVal_RO    cav1P2IfQ_;       // IF Q for cavity 1 probe 2, fixed 18.17
    ScalVal       cav1P2ChanSel_;   // Channel Select for cavity 1 probe 2, unsigned fixed 4.0
    ScalVal_RO    cav1P2DCReal_;    // DC real part for cavity 1, probe 2, fixed 21.19
    ScalVal_RO    cav1P2DCImage_;   // DC imaginary part for cavity 1, probe 2, fixed 21.19
    ScalVal       cav1P2WindowStart_;    // Integration window start for cavity 1, probe 2, unsigned fixed 16.0
    ScalVal       cav1P2WindowStop_;     // Integration window stop for cavity 1, probe 2,  unsigned fixed 16.0
    ScalVal_RO    cav1P2DCFreq_;    // DC frequency for cavity 1, probe 2, fixed 26.0
    ScalVal_RO    cav1P2IntegI_;    // Integrated I for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2IntegQ_;    // Integrated Q for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2OutPhase_;     // Output Phase for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2OutAmpl_;      // Output amplitude for cavity 1, probe 2, fixed 18.17

    ScalVal       cav1FreqEvalStart_;
    ScalVal       cav1FreqEvalStop_;

    ScalVal_RO    cav1P2CompPhase_;     // Comparison phase for cavity 1, probe 2, fixed 18.17

    ScalVal       cav1RegLatchPt_;

    ScalVal       cav1P2CalibCoeff_;    // calibration coefficient (TBD)
    ScalVal       cav1P2PhaseOffset_;   // Phase Offset (TBD)
    ScalVal       cav1P2Weight_;        // Weights (TBD)

    /* cavity 2 */
    /* probe 1 */

    ScalVal_RO    cav2P1IfAmpl_;    // IF amplitude for cavity 2 probe 1, fixed 18.17
    ScalVal_RO    cav2P1IfPhase_;   // IF phase for cavity 2 probe 1, fixed 18.17
    ScalVal_RO    cav2P1IfI_;       // IF I for cavity 2 probe 1, fixed 18.17
    ScalVal_RO    cav2P1IfQ_;       // IF Q for cavity 2 probe 1, fixed 18.17
    ScalVal       cav2P1ChanSel_;   // Channel Select for cavity 2 probe 1, unsigned fixed 4.0
    ScalVal_RO    cav2P1DCReal_;    // DC real part for cavity 2, probe 1, fixed 21.19
    ScalVal_RO    cav2P1DCImage_;   // DC imaginary part for cavity 2, probe 1, fixed 21.19
    ScalVal       cav2P1WindowStart_;    // Integration window start for cavity 2, probe 1, unsigned fixed 16.0
    ScalVal       cav2P1WindowStop_;     // Integration window stop for cavity 2, probe 1,  unsigned fixed 16.0
    ScalVal_RO    cav2P1DCFreq_;    // DC frequency for cavity 2, probe 1, fixed 26.0
    ScalVal_RO    cav2P1IntegI_;    // Integrated I for cavity 2, probe 1, fixed 18.17
    ScalVal_RO    cav2P1IntegQ_;    // Integrated Q for cavity 2, probe 1, fixed 18.17
    ScalVal_RO    cav2P1OutPhase_;     // Output Phase for cavity 2, probe 1, fixed 18.17
    ScalVal_RO    cav2P1OutAmpl_;      // Output amplitude for cavity 2, probe 1, fixed 18.17

    ScalVal_RO    cav2P1CompPhase_;     // Comparison phase for cavity 1, probe 1, fixed 18.17

    ScalVal       cav2P1CalibCoeff_;    // calibration coefficient (TBD)
    ScalVal       cav2P1PhaseOffset_;   // Phase Offset (TBD)
    ScalVal       cav2P1Weight_;        // Weights (TBD)
    // NCO
    ScalVal       cav2NCOPhaseAdj_;    // NCO phase adjust for cavity 2, unsigned fixed 29.29

    /* probe 2 */

    ScalVal_RO    cav2P2IfAmpl_;    // IF amplitude for cavity 2 probe 2, fixed 18.17
    ScalVal_RO    cav2P2IfPhase_;   // IF phase for cavity 2 probe 2, fixed 18.17
    ScalVal_RO    cav2P2IfI_;       // IF I for cavity 2 probe 2, fixed 18.17
    ScalVal_RO    cav2P2IfQ_;       // IF Q for cavity 2 probe 2, fixed 18.17
    ScalVal       cav2P2ChanSel_;   // Channel Select for cavity 2 probe 2, unsigned fixed 4.0
    ScalVal_RO    cav2P2DCReal_;    // DC real part for cavity 2, probe 2, fixed 21.19
    ScalVal_RO    cav2P2DCImage_;   // DC imaginary part for cavity 2, probe 2, fixed 21.19
    ScalVal       cav2P2WindowStart_;    // Integration window start for cavity 2, probe 2, unsigned fixed 16.0
    ScalVal       cav2P2WindowStop_;     // Integration window stop for cavity 2, probe 2,  unsigned fixed 16.0
    ScalVal_RO    cav2P2DCFreq_;    // DC frequency for cavity 2, probe 2, fixed 26.0
    ScalVal_RO    cav2P2IntegI_;    // Integrated I for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2IntegQ_;    // Integrated Q for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2OutPhase_;     // Output Phase for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2OutAmpl_;      // Output amplitude for cavity 2, probe 2, fixed 18.17

    ScalVal       cav2FreqEvalStart_;
    ScalVal       cav2FreqEvalStop_;

    ScalVal_RO    cav2P2CompPhase_;     // Comparison phase for cavity 2, probe 2, fixed 18.17

    ScalVal       cav2RegLatchPt_;

    ScalVal       cav2P2CalibCoeff_;    // calibration coefficient (TBD)
    ScalVal       cav2P2PhaseOffset_;   // Phase Offset (TBD)
    ScalVal       cav2P2Weight_;        // Weights (TBD)



public:
    CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie);

    virtual void getVersion(int32_t *version);

    /* config for reference */
    virtual void setRefSel(uint32_t channel);

    /* Waveform data Selector */
    virtual void setWfDataSel(int index, uint32_t sel);

    /* config for cavity and probe */
    virtual uint32_t setNCO(int cavity, double v);
    virtual void setChanSel(int cavity, int probe, uint32_t channel);
    virtual void setWindowStart(int cavity, int probe, uint32_t start);
    virtual void setWindowEnd(int cavity, int probe, uint32_t end);
    virtual void setFreqEvalStart(int cavity, uint32_t start);
    virtual void setFreqEvalEnd(int cavity, uint32_t end);
    virtual void setRegLatchPoint(int cavity, uint32_t point);
    virtual uint32_t setCalibCoeff(int cavity, int probe, double v);
    virtual uint32_t setPhaseOffset(int cavity, int probe, double v);
    virtual uint32_t setWeight(int cavity, int probe, double v);

    /* monitor for reference */
    virtual double getRefAmpl(int32_t *raw);
    virtual double getRefPhase(int32_t *raw);
    virtual double getRefI(int32_t *raw);
    virtual double getRefQ(int32_t *raw);

    /* monitor for cavity and probe */
    virtual double getIfAmpl(int cavity, int probe, int32_t *raw);
    virtual double getIfPhase(int cavity, int probe, int32_t *raw);
    virtual double getIfI(int cavity, int probe, int32_t *raw);
    virtual double getIfQ(int cavity, int probe, int32_t *raw);
    virtual double getDCReal(int cavity, int probe, int32_t *raw);
    virtual double getDCImage(int cavity, int probe, int32_t *raw);
    virtual double getDCFreq(int cavity, int probe, int32_t *raw);
    virtual double getIntegI(int cavity, int probe, int32_t *raw);
    virtual double getIntegQ(int cavity, int probe, int32_t *raw);
    virtual double getOutPhase(int cavity, int probe, int32_t *raw);
    virtual double getOutAmpl(int cavity, int probe, int32_t *raw);
    virtual double getCompPhase(int cavity, int probe, int32_t *raw);
    virtual double getPhaseOffset(int cavity, int probe, int32_t *raw);
    virtual double getWeight(int cavity, int probe, int32_t *raw);
};


pcavFw IpcavFw::create(Path p)
{
    return IEntryAdapt::check_interface<pcavFwAdapt, DevImpl> (p);
}

CpcavFwAdapt::CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie):
    IEntryAdapt(k, p, ie),
    pPcavReg_(p->findByName("AppTop/AppCore/Sysgen/PcavReg")),
    pDiagBus_(p->findByName("AppTop/AppCore/AppDiagnBus")),

    version_(        IScalVal_RO::create(pPcavReg_->findByName("version"))),

    /* rf reference */
    rfRefAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("rfRefAmpl"))),
    rfRefPhase_(     IScalVal_RO::create(pPcavReg_->findByName("rfRefPhase"))),
    rfRefI_(         IScalVal_RO::create(pPcavReg_->findByName("rfRefI"))),
    rfRefQ_(         IScalVal_RO::create(pPcavReg_->findByName("rfRefQ"))),
    rfRefSel_(       IScalVal   ::create(pPcavReg_->findByName("rfRefSel"))),


    /* cavity 1 */
    /* probe 1 */

    cav1P1IfAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P1IfAmpl"))),
    cav1P1IfPhase_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P1IfPhase"))),
    cav1P1IfI_(         IScalVal_RO::create(pPcavReg_->findByName("cav1P1IfI"))),
    cav1P1IfQ_(         IScalVal_RO::create(pPcavReg_->findByName("cav1P1IfQ"))),
    cav1P1ChanSel_(     IScalVal   ::create(pPcavReg_->findByName("cav1P1ChanSel"))),
    cav1P1DCReal_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P1DCReal"))),
    cav1P1DCImage_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P1DCImage"))),
    cav1P1WindowStart_( IScalVal   ::create(pPcavReg_->findByName("cav1P1WindowStart"))),
    cav1P1WindowStop_(  IScalVal   ::create(pPcavReg_->findByName("cav1P1WindowStop"))),
    cav1P1DCFreq_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P1DCFreq"))),
    cav1P1IntegI_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P1IntegI"))),
    cav1P1IntegQ_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P1IntegQ"))),
    cav1P1OutPhase_(    IScalVal_RO::create(pPcavReg_->findByName("cav1P1OutPhase"))),
    cav1P1OutAmpl_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P1OutAmpl"))),

    cav1P1CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav1P1CompPhase"))),

    cav1P1CalibCoeff_(  IScalVal::create(pPcavReg_->findByName("cav1P1CalibCoeff"))),
    cav1P1PhaseOffset_( IScalVal::create(pDiagBus_->findByName("Cavity0Probe0PhaseOffset"))),
    cav1P1Weight_(      IScalVal::create(pDiagBus_->findByName("Cavity0Probe0Weight"))),

    /* NCO for cavity 1 */

    cav1NCOPhaseAdj_(  IScalVal    ::create(pPcavReg_->findByName("cav1NCOPhaseAdj"))),

    /* probe 2 */

    cav1P2IfAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P2IfAmpl"))),
    cav1P2IfPhase_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P2IfPhase"))),
    cav1P2IfI_(         IScalVal_RO::create(pPcavReg_->findByName("cav1P2IfI"))),
    cav1P2IfQ_(         IScalVal_RO::create(pPcavReg_->findByName("cav1P2IfQ"))),
    cav1P2ChanSel_(     IScalVal   ::create(pPcavReg_->findByName("cav1P2ChanSel"))),
    cav1P2DCReal_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P2DCReal"))),
    cav1P2DCImage_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P2DCImage"))),
    cav1P2WindowStart_( IScalVal   ::create(pPcavReg_->findByName("cav1P2WindowStart"))),
    cav1P2WindowStop_(  IScalVal   ::create(pPcavReg_->findByName("cav1P2WindowStop"))),
    cav1P2DCFreq_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P2DCFreq"))),
    cav1P2IntegI_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P2IntegI"))),
    cav1P2IntegQ_(      IScalVal_RO::create(pPcavReg_->findByName("cav1P2IntegQ"))),
    cav1P2OutPhase_(    IScalVal_RO::create(pPcavReg_->findByName("cav1P2OutPhase"))),
    cav1P2OutAmpl_(     IScalVal_RO::create(pPcavReg_->findByName("cav1P2OutAmpl"))),

    cav1FreqEvalStart_( IScalVal   ::create(pPcavReg_->findByName("cav1FreqEvalStart"))),
    cav1FreqEvalStop_(  IScalVal   ::create(pPcavReg_->findByName("cav1FreqEvalStop"))),

    cav1P2CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav1P2CompPhase"))),

    cav1RegLatchPt_(    IScalVal   ::create(pPcavReg_->findByName("cav1RegLatchPt"))),

    cav1P2CalibCoeff_(  IScalVal::create(pPcavReg_->findByName("cav1P2CalibCoeff"))),
    cav1P2PhaseOffset_( IScalVal::create(pDiagBus_->findByName("Cavity0Probe1PhaseOffset"))),
    cav1P2Weight_(      IScalVal::create(pDiagBus_->findByName("Cavity0Probe1Weight"))),


    /* cavity 2 */
    /* probe 1 */

    cav2P1IfAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P1IfAmpl"))),
    cav2P1IfPhase_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P1IfPhase"))),
    cav2P1IfI_(         IScalVal_RO::create(pPcavReg_->findByName("cav2P1IfI"))),
    cav2P1IfQ_(         IScalVal_RO::create(pPcavReg_->findByName("cav2P1IfQ"))),
    cav2P1ChanSel_(     IScalVal   ::create(pPcavReg_->findByName("cav2P1ChanSel"))),
    cav2P1DCReal_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P1DCReal"))),
    cav2P1DCImage_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P1DCImage"))),
    cav2P1WindowStart_( IScalVal   ::create(pPcavReg_->findByName("cav2P1WindowStart"))),
    cav2P1WindowStop_(  IScalVal   ::create(pPcavReg_->findByName("cav2P1WindowStop"))),
    cav2P1DCFreq_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P1DCFreq"))),
    cav2P1IntegI_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P1IntegI"))),
    cav2P1IntegQ_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P1IntegQ"))),
    cav2P1OutPhase_(    IScalVal_RO::create(pPcavReg_->findByName("cav2P1OutPhase"))),
    cav2P1OutAmpl_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P1OutAmpl"))),

    cav2P1CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav2P1CompPhase"))),

    cav2P1CalibCoeff_(  IScalVal   ::create(pPcavReg_->findByName("cav2P1CalibCoeff"))),
    cav2P1PhaseOffset_( IScalVal::create(pDiagBus_->findByName("Cavity1Probe0PhaseOffset"))),
    cav2P1Weight_(      IScalVal::create(pDiagBus_->findByName("Cavity1Probe0Weight"))),

    /* NCO for cavity 2 */

    cav2NCOPhaseAdj_(  IScalVal    ::create(pPcavReg_->findByName("cav2NCOPhaseAdj"))),

    /* probe 2 */

    cav2P2IfAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P2IfAmpl"))),
    cav2P2IfPhase_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P2IfPhase"))),
    cav2P2IfI_(         IScalVal_RO::create(pPcavReg_->findByName("cav2P2IfI"))),
    cav2P2IfQ_(         IScalVal_RO::create(pPcavReg_->findByName("cav2P2IfQ"))),
    cav2P2ChanSel_(     IScalVal   ::create(pPcavReg_->findByName("cav2P2ChanSel"))),
    cav2P2DCReal_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P2DCReal"))),
    cav2P2DCImage_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P2DCImage"))),
    cav2P2WindowStart_( IScalVal   ::create(pPcavReg_->findByName("cav2P2WindowStart"))),
    cav2P2WindowStop_(  IScalVal   ::create(pPcavReg_->findByName("cav2P2WindowStop"))),
    cav2P2DCFreq_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P2DCFreq"))),
    cav2P2IntegI_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P2IntegI"))),
    cav2P2IntegQ_(      IScalVal_RO::create(pPcavReg_->findByName("cav2P2IntegQ"))),
    cav2P2OutPhase_(    IScalVal_RO::create(pPcavReg_->findByName("cav2P2OutPhase"))),
    cav2P2OutAmpl_(     IScalVal_RO::create(pPcavReg_->findByName("cav2P2OutAmpl"))),

    cav2FreqEvalStart_( IScalVal   ::create(pPcavReg_->findByName("cav2FreqEvalStart"))),
    cav2FreqEvalStop_(  IScalVal   ::create(pPcavReg_->findByName("cav2FreqEvalStop"))),

    cav2P2CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav2P2CompPhase"))),

    cav2RegLatchPt_(    IScalVal   ::create(pPcavReg_->findByName("cav2RegLatchPt"))),

    cav2P2CalibCoeff_(  IScalVal::create(pPcavReg_->findByName("cav2P2CalibCoeff"))),
    cav2P2PhaseOffset_( IScalVal::create(pDiagBus_->findByName("Cavity1Probe1PhaseOffset"))),
    cav2P2Weight_(      IScalVal::create(pDiagBus_->findByName("Cavity1Probe1Weight")))

{
    char name[80];

     for(int i = 0; i < 8; i++) {
         sprintf(name, "wfData%dSel", i);
         wfDataSel_[i] = IScalVal::create(pPcavReg_->findByName(name));
     }
}

void CpcavFwAdapt::getVersion(int32_t *version)
{
    CPSW_TRY_CATCH(version_->getVal((uint32_t*) version));
}

//
//
/* config for  reference */
//
//

void CpcavFwAdapt::setRefSel(uint32_t channel)
{
    CPSW_TRY_CATCH(rfRefSel_->setVal(channel));
}

//
//
/* waveform data selector */
//
//

void CpcavFwAdapt::setWfDataSel(int index, uint32_t sel)
{
    CPSW_TRY_CATCH(wfDataSel_[index]->setVal(sel));
}

//
//
/* config for cavity and probe */
//
//

uint32_t CpcavFwAdapt::setNCO(int cavity, double v)
{
    uint32_t  out = nco(v);

    switch(cavity) {
        case 0:    // cavity 0
            CPSW_TRY_CATCH(cav1NCOPhaseAdj_->setVal(out));
            break;
        case 1:    // cavity 1
            CPSW_TRY_CATCH(cav2NCOPhaseAdj_->setVal(out));
            break;
    }

    return out;
}


void CpcavFwAdapt::setChanSel(int cavity, int probe, uint32_t channel)
{
    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:         // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1ChanSel_->setVal(channel));
                    break;
                case 1:         // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2ChanSel_->setVal(channel));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:         // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1ChanSel_->setVal(channel));
                    break;
                case 1:         // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2ChanSel_->setVal(channel));
                    break;
            }
            break;
    }
}

void CpcavFwAdapt::setWindowStart(int cavity, int probe, uint32_t start)
{
    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1WindowStart_->setVal(start));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2WindowStart_->setVal(start));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1WindowStart_->setVal(start));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2WindowStart_->setVal(start));
                    break;
            }
            break;
    }
}

void CpcavFwAdapt::setWindowEnd(int cavity, int probe, uint32_t end)
{

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1WindowStop_->setVal(end));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2WindowStop_->setVal(end));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1WindowStop_->setVal(end));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2WindowStop_->setVal(end));
                    break;
            }
            break;
    }
}

void CpcavFwAdapt::setFreqEvalStart(int cavity, uint32_t start)
{
    switch(cavity) {
        case 0:  // cavity 0
            CPSW_TRY_CATCH(cav1FreqEvalStart_->setVal(start));
            break;
        case 1:  // cavity 1
            CPSW_TRY_CATCH(cav2FreqEvalStart_->setVal(start));
            break;
    }
}

void CpcavFwAdapt::setFreqEvalEnd(int cavity, uint32_t end)
{
    switch(cavity) {
        case 0:    // cavity 0
            CPSW_TRY_CATCH(cav1FreqEvalStop_->setVal(end));
            break;
        case 1:    // cavity 1
            CPSW_TRY_CATCH(cav2FreqEvalStop_->setVal(end));
            break;
    }
}

void CpcavFwAdapt::setRegLatchPoint(int cavity, uint32_t point)
{
    switch(cavity) {
        case 0:    // cavity 0
            CPSW_TRY_CATCH(cav1RegLatchPt_->setVal(point));
            break;
        case 1:    // cavity 1
            CPSW_TRY_CATCH(cav2RegLatchPt_->setVal(point));
            break;
    }
}

uint32_t CpcavFwAdapt::setCalibCoeff(int cavity, int probe, double v)
{
    uint32_t out = (v * ((1<< 17)-1));

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1CalibCoeff_->setVal(out));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2CalibCoeff_->setVal(out));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1CalibCoeff_->setVal(out));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2CalibCoeff_->setVal(out));
                    break;
            }
            break;
    }

    return out;
}

uint32_t CpcavFwAdapt::setPhaseOffset(int cavity, int probe, double v)
{
    uint32_t out = (v * ((1<<15)-1));

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1PhaseOffset_->setVal(out));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2PhaseOffset_->setVal(out));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1PhaseOffset_->setVal(out));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2PhaseOffset_->setVal(out));
                    break;
            }
            break;
    }
    return out;
}

uint32_t CpcavFwAdapt::setWeight(int cavity, int probe, double v)
{
    uint32_t out = (v * (1<<1));

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1Weight_->setVal(out));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2Weight_->setVal(out));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1Weight_->setVal(out));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2Weight_->setVal(out));
                    break;
            }
            break;
    }
    return out;
}

//
//
/* monitor for reference */
//
//

double CpcavFwAdapt::getRefAmpl(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefAmpl_->getVal((uint32_t*) raw));

    v = _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefPhase(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefPhase_->getVal((uint32_t*) raw));

    v = 180. *  _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefI(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefI_->getVal((uint32_t*) raw));

    v = _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefQ(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefQ_->getVal((uint32_t*) raw));

    v = _FIX_18_17(*raw);

    return v;
}



//
//
/* monitor for cavity and probe */
//
//

double CpcavFwAdapt::getIfAmpl(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IfAmpl_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IfAmpl_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IfAmpl_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IfAmpl_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getIfPhase(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IfPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IfPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IfPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IfPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = 180. * _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getIfI(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IfI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IfI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IfI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IfI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getIfQ(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IfQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IfQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IfQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IfQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }
  
    v = _FIX_18_17(*raw);

    return v;
}

double CpcavFwAdapt::getDCReal(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1DCReal_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2DCReal_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1DCReal_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2DCReal_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_16(*raw);

    return v;
}

double CpcavFwAdapt::getDCImage(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1DCImage_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2DCImage_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1DCImage_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2DCImage_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_16(*raw);

    return v;
}

double CpcavFwAdapt::getDCFreq(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1DCFreq_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2DCFreq_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1DCFreq_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2DCFreq_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_32_18(*raw);

    return v;
}

double CpcavFwAdapt::getIntegI(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IntegI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IntegI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IntegI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IntegI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_16(*raw);

    return v;
}

double CpcavFwAdapt::getIntegQ(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1IntegQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2IntegQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1IntegQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2IntegQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_16(*raw);

    return v;
}

double CpcavFwAdapt::getOutPhase(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1OutPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2OutPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1OutPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2OutPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = 180. * _FIX_18_15(*raw);

    return v;
}

double CpcavFwAdapt::getOutAmpl(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1OutAmpl_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2OutAmpl_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1OutAmpl_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2OutAmpl_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_18_16(*raw);

    return v;
}



double CpcavFwAdapt::getCompPhase(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1CompPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2CompPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1CompPhase_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2CompPhase_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = 180. * _FIX_18_15(*raw);

    return v;
}


double CpcavFwAdapt::getPhaseOffset(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1PhaseOffset_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2PhaseOffset_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1PhaseOffset_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2PhaseOffset_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = 180. * _FIX_18_15(*raw);

    return v;
}

double CpcavFwAdapt::getWeight(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1Weight_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2Weight_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1Weight_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2Weight_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = _FIX_2_1(*raw);

    return v;
}

