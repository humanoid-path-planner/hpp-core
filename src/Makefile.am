include $(top_srcdir)/build-aux/init.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

lib_LTLIBRARIES = libhppCore.la
libhppCore_la_SOURCES = 	\
	hppColPair.cpp 		\
	hppPlanner.cpp		\
	hppProblem.cpp

libhppCore_la_LIBADD = @KWSPLUS_LIBS@
libhppCore_la_LDFLAGS = -release ${PACKAGE_VERSION}

if BODY
 AM_CPPFLAGS += -I$(top_srcdir)/include/hppModel
 libhppCore_la_SOURCES += hppBody.cpp
else
 AM_CPPFLAGS += @HPPMODEL_CFLAGS@
 libhppCore_la_LIBADD += @HPPMODEL_LIBS@
 libhppCore_la_SOURCES += 	\
	parser.cc \
	joint.cc \
	joint.hh \
	freeflyer-joint.hh \
	rotation-joint.hh \
	translation-joint.hh
endif

# Add kwsPlus CPPFLAGS.
AM_CPPFLAGS += @KWSPLUS_CFLAGS@
