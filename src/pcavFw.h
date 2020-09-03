#ifndef _PCAVFW_H
#define _PCAVFW_H

#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>

class IpcavFw;
typedef shared_ptr <IpcavFw> pcavFw;

class IpcavFw : public virtual IEntry {
public:
    static pcavFw create(Path p);

    virtual void getVersion(int32_t *version) = 0;
    virtual void setRefSel(uint32_t channel) = 0;

    virtual void setWfDataSel(int index, uint32_t sel) = 0;


    virtual uint32_t setNCO(int cavity, double v) = 0;
    virtual void setChanSel(int cavity, int probe, uint32_t channel) = 0;
    virtual void setWindowStart(int cavity, int probe, uint32_t start) = 0;
    virtual void setWindowEnd(int cavity, int probe, uint32_t end) = 0;
    virtual void setFreqEvalStart(int cavity, uint32_t start) = 0;
    virtual void setFreqEvalEnd(int cavity, uint32_t end) = 0;
    virtual void setRegLatchPoint(int cavity, uint32_t point) = 0;
    virtual uint32_t setCalibCoeff(int cavity, int probe, double v) = 0;

    virtual double getRefAmpl(int32_t *raw) = 0;
    virtual double getRefPhase(int32_t *raw) = 0;
    virtual double getRefI(int32_t *raw) = 0;
    virtual double getRefQ(int32_t *raw) = 0;

    virtual double getIfAmpl(int cavity, int probe, int32_t *raw) = 0;
    virtual double getIfPhase(int cavity, int probe, int32_t *raw) = 0;
    virtual double getIfI(int cavity, int probe, int32_t *raw) = 0;
    virtual double getIfQ(int cavity, int probe, int32_t *raw) = 0;
    virtual double getDCReal(int cavity, int probe, int32_t *raw) = 0;
    virtual double getDCImage(int cavity, int probe, int32_t *raw) = 0;
    virtual double getDCFreq(int cavity, int probe, int32_t *raw) = 0;
    virtual double getIntegI(int cavity, int probe, int32_t *raw) = 0;
    virtual double getIntegQ(int cavity, int probe, int32_t *raw) = 0;
    virtual double getOutPhase(int cavity, int probe, int32_t *raw) = 0;
    virtual double getOutAmpl(int cavity, int probe, int32_t *raw) = 0;
    virtual double getCompPhase(int cavity, int probe, int32_t *raw) = 0;
    
};

#endif /* _PCAVFW_H */
