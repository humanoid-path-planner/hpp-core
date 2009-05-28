# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#
ACLOCAL_AMFLAGS = -I build-aux

SUBDIRS = \
	include \
	src	\
	doc

hppCoredir = $(libdir)/pkgconfig
hppCore_DATA = hppCore.pc

# -- Generate documentation
.PHONY: doc html
doc html:
	cd doc && $(MAKE) $(AM_MAKEFLAGS) $@
