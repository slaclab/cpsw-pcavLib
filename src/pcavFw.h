#ifndef _PCAVFW_H
#define _PCAVFW_H

#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>

class IpcavFw;
typedef shared_ptr <IpcavFw> pcavFw;

class IpcavFw : public virtual IEntry {
public:
    static pcavFw create(Path p);

    virtual void setNCO(int cavity, double v) = 0;
    virtual void setChanSel(int cavity, int probe, uint32_t channel) = 0;
};

#endif /* _PCAVFW_H */
