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

inline static double fixed18_17(uint32_t v)
{
    double out = (double) (v & 0x0003ffff) / (double) (0x0001ffff);

    if(v & 0x00020000) out -= 1.;

    return out;
}

inline static double fixed21_19(uint32_t v)
{
    double out = (double) (v & 0x001fffff) / (double) (0x0007ffff);

    if(v & 0x00100000) out -= 2.;

    return out;
}

inline static double fixed26_0(uint32_t v)
{
    double out = (double) (v & 0x03ffffff);

    if(v & 0x02000000) out -= (double) (0x02000000);

    return out;
}

inline static double ufixed29_29(uint32_t v)
{
    double out = (double) (v & 0x1fffffff) / (double) (0x1fffffff);

    return out;
}

inline static uint32_t ufixed29_29(double v)
{
    uint32_t out = (uint32_t) (v * (double)(0x1fffffff));

    return out;
}



class CpcavFwAdapt;
typedef shared_ptr<CpcavFwAdapt> pcavFwAdapt;

class CpcavFwAdapt : public IpcavFw, public IEntryAdapt {
private:
protected:
    Path pPcavReg_;      // pcav register path

    ScalVal_RO    version_;      // pcav firmware version

    /* rf reference */
    ScalVal_RO    rfRefAmpl_;    // RF reference amplitude,  fixed point 18.17
    ScalVal_RO    rfRefPhase_;   // RF reference phase, fixed point 18.17
    ScalVal_RO    rfRefI_;       // RF reference I, fixed point 18.17
    ScalVal_RO    rfRefQ_;       // RF reference Q, fixed point 18.17
    ScalVal       rfRefSel_;     // RF reference selection, unsigned fixed 4.0
    ScalVal       refWindowStart_;      // Integration window start for reference, unsigned fixed 16.0
    ScalVal       refWindowStop_;       // Integration window stop for reference,  unsigned fixed 16.0
    
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
    ScalVal_RO    cav1P1CompI_;     // Comparison I for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1CompQ_;     // comparison Q for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1CompPhase_;     // Comparison phase for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav1P1IfWf_;      // If waveform (TBD, space holder)
    ScalVal       cav1P1CalibCoeff_;    // calibration coefficient (TBD)
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
    ScalVal_RO    cav1P2CompI_;     // Comparison I for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2CompQ_;     // comparison Q for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2CompPhase_;     // Comparison phase for cavity 1, probe 2, fixed 18.17
    ScalVal_RO    cav1P2IfWf_;      // If waveform (TBD, space holder)
    ScalVal       cav1P2CalibCoeff_;    // calibration coefficient (TBD)

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
    ScalVal_RO    cav2P1CompI_;     // Comparison I for cavity 2, probe 1, fixed 18.17
    ScalVal_RO    cav2P1CompQ_;     // comparison Q for cavity 2, probe 1, fixed 18.17
    ScalVal_RO    cav2P1CompPhase_;     // Comparison phase for cavity 1, probe 1, fixed 18.17
    ScalVal_RO    cav2P1IfWf_;      // If waveform (TBD, space holder)
    ScalVal       cav2P1CalibCoeff_;    // calibration coefficient (TBD)
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
    ScalVal_RO    cav2P2CompI_;     // Comparison I for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2CompQ_;     // comparison Q for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2CompPhase_;     // Comparison phase for cavity 2, probe 2, fixed 18.17
    ScalVal_RO    cav2P2IfWf_;      // If waveform (TBD, space holder)
    ScalVal       cav2P2CalibCoeff_;    // calibration coefficient (TBD)



public:
    CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie);

    virtual void getVersion(int32_t *version);

    /* config for reference */
    virtual void setRefSel(uint32_t channel);
    virtual void setRefWindowStart(uint32_t start);
    virtual void setRefWindowEnd(uint32_t end);

    /* config for cavity and probe */
    virtual void setNCO(int cavity, double v);
    virtual void setChanSel(int cavity, int probe, uint32_t channel);
    virtual void setWindowStart(int cavity, int probe, uint32_t start);
    virtual void setWindowEnd(int cavity, int probe, uint32_t end);
    virtual void setCalibCoeff(int cavity, int probe, uint32_t v);

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
    virtual double getCompI(int cavity, int probe, int32_t *raw);
    virtual double getCompQ(int cavity, int probe, int32_t *raw);
    virtual double getCompPhase(int cavity, int probe, int32_t *raw);
};


pcavFw IpcavFw::create(Path p)
{
    return IEntryAdapt::check_interface<pcavFwAdapt, DevImpl> (p);
}

CpcavFwAdapt::CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie):
    IEntryAdapt(k, p, ie),
    pPcavReg_(p->findByName("")),

    version_(        IScalVal_RO::create(pPcavReg_->findByName("version"))),

    /* rf reference */
    rfRefAmpl_(      IScalVal_RO::create(pPcavReg_->findByName("rfRefAmpl"))),
    rfRefPhase_(     IScalVal_RO::create(pPcavReg_->findByName("rfRefPhase"))),
    rfRefI_(         IScalVal_RO::create(pPcavReg_->findByName("rfRefI"))),
    rfRefQ_(         IScalVal_RO::create(pPcavReg_->findByName("rfRefQ"))),
    rfRefSel_(       IScalVal   ::create(pPcavReg_->findByName("rfRefSel"))),
    refWindowStart_( IScalVal   ::create(pPcavReg_->findByName("refWindowStart"))),
    refWindowStop_(  IScalVal   ::create(pPcavReg_->findByName("refWindowStop"))),

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
    cav1P1CompI_(       IScalVal_RO::create(pPcavReg_->findByName("cav1P1CompI"))),
    cav1P1CompQ_(       IScalVal_RO::create(pPcavReg_->findByName("cav1P1CompQ"))),
    cav1P1CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav1P1CompPhase"))),
    cav1P1IfWf_(        IScalVal_RO::create(pPcavReg_->findByName("cav1P1IfWf"))),
    cav1P1CalibCoeff_(  IScalVal   ::create(pPcavReg_->findByName("cav1P1CalibCoeff"))),

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
    cav1P2CompI_(       IScalVal_RO::create(pPcavReg_->findByName("cav1P2CompI"))),
    cav1P2CompQ_(       IScalVal_RO::create(pPcavReg_->findByName("cav1P2CompQ"))),
    cav1P2CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav1P2CompPhase"))),
    cav1P2IfWf_(        IScalVal_RO::create(pPcavReg_->findByName("cav1P2IfWf"))),
    cav1P2CalibCoeff_(  IScalVal   ::create(pPcavReg_->findByName("cav1P2CalibCoeff"))),


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
    cav2P1CompI_(       IScalVal_RO::create(pPcavReg_->findByName("cav2P1CompI"))),
    cav2P1CompQ_(       IScalVal_RO::create(pPcavReg_->findByName("cav2P1CompQ"))),
    cav2P1CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav2P1CompPhase"))),
    cav2P1IfWf_(        IScalVal_RO::create(pPcavReg_->findByName("cav2P1IfWf"))),
    cav2P1CalibCoeff_(  IScalVal   ::create(pPcavReg_->findByName("cav2P1CalibCoeff"))),

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
    cav2P2CompI_(       IScalVal_RO::create(pPcavReg_->findByName("cav2P2CompI"))),
    cav2P2CompQ_(       IScalVal_RO::create(pPcavReg_->findByName("cav2P2CompQ"))),
    cav2P2CompPhase_(   IScalVal_RO::create(pPcavReg_->findByName("cav2P2CompPhase"))),
    cav2P2IfWf_(        IScalVal_RO::create(pPcavReg_->findByName("cav2P2IfWf"))),
    cav2P2CalibCoeff_(  IScalVal   ::create(pPcavReg_->findByName("cav2P2CalibCoeff")))

{
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

void CpcavFwAdapt::setRefWindowStart(uint32_t start)
{
    CPSW_TRY_CATCH(refWindowStart_->setVal(start));
}

void CpcavFwAdapt::setRefWindowEnd(uint32_t end)
{
    CPSW_TRY_CATCH(refWindowStop_->setVal(end));
}

//
//
/* config for cavity and probe */
//
//

void CpcavFwAdapt::setNCO(int cavity, double v)
{
    uint32_t  out = ufixed29_29(v);

    switch(cavity) {
        case 0:    // cavity 0
            CPSW_TRY_CATCH(cav1NCOPhaseAdj_->setVal(out));
            break;
        case 1:    // cavity 1
            CPSW_TRY_CATCH(cav2NCOPhaseAdj_->setVal(out));
            break;
    }
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

void CpcavFwAdapt::setCalibCoeff(int cavity, int probe, uint32_t v)
{
    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1CalibCoeff_->setVal(v));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2CalibCoeff_->setVal(v));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1CalibCoeff_->setVal(v));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2CalibCoeff_->setVal(v));
                    break;
            }
            break;
    }
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
    v = fixed18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefPhase(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefPhase_->getVal((uint32_t*) raw));
    v = 180. * fixed18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefI(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefI_->getVal((uint32_t*) raw));
    v = fixed18_17(*raw);

    return v;
}

double CpcavFwAdapt::getRefQ(int32_t *raw)
{
    double v;

    CPSW_TRY_CATCH(rfRefQ_->getVal((uint32_t*) raw));
    v = fixed18_17(*raw);

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

    v = fixed18_17(*raw);

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

    v = 180. * fixed18_17(*raw);

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

    v = fixed18_17(*raw);

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
  
    v = fixed18_17(*raw);

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

    v = fixed21_19(*raw);

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

    v = fixed21_19(*raw);

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

    v = fixed26_0(*raw);

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

    v = fixed18_17(*raw);

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

    v = fixed18_17(*raw);

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

    v = 180. * fixed18_17(*raw);

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

    v = fixed18_17(*raw);

    return v;
}

double CpcavFwAdapt::getCompI(int cavity, int probe, int32_t *raw)
{
    double v;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1CompI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2CompI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1CompI_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2CompI_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = fixed18_17(*raw);

    return v;
}

double CpcavFwAdapt::getCompQ(int cavity, int probe, int32_t *raw)
{
    double v = 0.;

    switch(cavity) {
        case 0:
            switch(probe) {
                case 0:    // cavity 0, probe 0
                    CPSW_TRY_CATCH(cav1P1CompQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 0, probe 1
                    CPSW_TRY_CATCH(cav1P2CompQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
        case 1:
            switch(probe) {
                case 0:    // cavity 1, probe 0
                    CPSW_TRY_CATCH(cav2P1CompQ_->getVal((uint32_t*) raw));
                    break;
                case 1:    // cavity 1, probe 1
                    CPSW_TRY_CATCH(cav2P2CompQ_->getVal((uint32_t*) raw));
                    break;
            }
            break;
    }

    v = fixed18_17(*raw);

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

    v = 180. * fixed18_17(*raw);

    return v;
}


