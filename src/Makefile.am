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
	hppBody.cpp \
	hppDevice.cpp \
	hppColPair.cpp \
	hppProblem.cpp \
	hppPlanner.cpp

libhppCore_la_CPPFLAGS = -I$(srcdir)/../include @KWSPLUS_CFLAGS@
libhppCore_la_LIBADD = @KWSPLUS_LIBS@
