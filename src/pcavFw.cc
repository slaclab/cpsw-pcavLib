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


class CpcavFwAdapt;
typedef shared_ptr<CpcavFwAdapt> pcavFwAdapt;

class CpcavFwAdapt : public IpcavFw, public IEntryAdapt {
private:
protected:
public:
    CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie);
};


pcavFw IpcavFw::create(Path p)
{
    return IEntryAdapt::check_interface<pcavFwAdapt, DevImpl> (p);
}

CpcavFwAdapt::CpcavFwAdapt(Key &k, ConstPath p, shared_ptr<const CEntryImpl> ie):
    IEntryAdapt(k, p, ie)
{
}
