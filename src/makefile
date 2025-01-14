ifeq ($(PACKAGE_TOP),)
ifneq ($(EPICS_PACKAGE_TOP),)
PACKAGE_TOP=$(EPICS_PACKAGE_TOP)
else
$(error PACKAGE_TOP or EPICS_PACKAGE_TOP must be provided by the environment or on the command line)
endif
endif

CPSW_VERSION=R4.5.2

CPSW_DIR=$(PACKAGE_TOP)/cpsw/framework/$(CPSW_VERSION)/src

CCACHE_DISABLE=1

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

INSTALL_DIR=../

HEADERS += pcavFw.h
HEADERS += dacSigGenFw.h

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
