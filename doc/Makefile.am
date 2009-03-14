# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppCoredir = $(datadir)/doc/hppCore/html

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/hppCore.h \
	figures/archi.fig \
	figures/archi.png \
	figures/archi.pdf \
	pictures/HRP2.jpg


install: html-doc
	mkdir -p $(hppCoredir)
	$(INSTALL_DATA) html/* $(hppCoredir)
	mkdir -p $(hppCoredir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppCoredir)/pictures
	$(INSTALL_DATA) hppCore.doxytag $(hppCoredir)

html-doc: hpp.dox
	doxygen $<
