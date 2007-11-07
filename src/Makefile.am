# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libhppCore.la
libhppCore_la_SOURCES = \
	hppPolyhedron.cpp \
	hppBox.cpp \
	hppDevice.cpp \
	hppColPair.cpp \
	hppProblem.cpp \
	hppPlanner.cpp

if BODY
libhppCore_la_SOURCES += \
	hppBody.cpp 
endif

libhppCore_la_CPPFLAGS = -I$(srcdir)/../include @KWSPLUS_CFLAGS@
libhppCore_la_LDFLAGS =\
	@KWSPLUS_LIBS@	\
	-release ${PACKAGE_VERSION}

