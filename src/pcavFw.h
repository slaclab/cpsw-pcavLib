#ifndef _PCAVFW_H
#define _PCAVFW_H

#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>

class IpcavFw;
typedef shared_ptr <IpcavFw> pcavFw;

class IpcavFw : public virtual IEntry {
public:
    static pcavFw create(Path p);
};

#endif /* _PCAVFW_H */
