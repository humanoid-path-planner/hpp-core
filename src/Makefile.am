# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libhppCore.la
libhppCore_la_SOURCES = \
	hppColPair.cpp \
	hppProblem.cpp \
	hppPlanner.cpp

libhppCore_la_CPPFLAGS = -I$(srcdir)/../include 
libhppCore_la_LDFLAGS =\
	@KWSPLUS_LIBS@	\
	-release ${PACKAGE_VERSION}

if BODY
libhppCore_la_SOURCES += \
	hppBody.cpp 
libhppCore_la_CPPFLAGS += -I$(srcdir)/../include/hppModel
else
libhppCore_la_CPPFLAGS += @HPPMODEL_CFLAGS@
libhppCore_la_LDFLAGS += @HPPMODEL_LIBS@
endif

libhppCore_la_CPPFLAGS += @KWSPLUS_CFLAGS@
