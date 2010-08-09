include $(top_srcdir)/build-aux/init.mk

# --- Distribute and install headers.
nobase_include_HEADERS =			 	\
	hppCore/hppColPair.h 				\
	hppCore/hppPlanner.h				\
	hppCore/hppProblem.h

if BODY
 hppModeldir = $(includedir)/hppModel
 hppModel_HEADERS = hppModel/hppModel/hppBody.h
else
 nobase_include_HEADERS +=			 	\
	hpp/core/parser.hh \
	hpp/core/joint-properties.hh \
	hpp/core/freeflyer-joint.hh \
	hpp/core/rotation-joint.hh \
	hpp/core/translation-joint.hh \
	hpp/core/fwd.hh
endif



