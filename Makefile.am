# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

SUBDIRS = \
	include \
	src	\
	doc

hppCoredir = $(libdir)/pkgconfig
hppCore_DATA = hppCore.pc

install-data-hook: documentation

documentation:
	cd doc; $(MAKE) install
