CPSW_DIR=/afs/slac/g/lcls/package/cpsw/framework/R4.4.1/src

CCACHE_DISABLE=1

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

INSTALL_DIR=../

HEADERS += pcavFw.h
HEADERS += dacSigGen.h

pcavLib_SRCS  = pcavFw.cc
pcavLib_SRCS += dacSigGenFw.cc
pcavLib_LIBS  = $(CPSW_LIBS)

SHARED_LIBRARIES_YES += pcavLib
STATIC_LIBRARIES_YES += pcavLib

pcavLib_tst_SRCS = pcavFw_tst.cc
pcavLib_tst_LIBS = pcavLib
pcavLib_tst_LIBS += $(CPSW_LIBS)

PROGRAMS=pcavLib_tst

include $(CPSW_DIR)/rules.mak
