# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

.PHONY: doc html install-doxygen-html uninstall-doxygen-html

hppCoredir = $(datadir)/doc/hppCore/html

doc: html

html-local: hpp.dox
	$(DOXYGEN) hpp.dox

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/hppCore.h \
	figures/archi.fig \
	figures/archi.png \
	figures/archi.pdf \
	pictures/HRP2.jpg


CLEANFILES = hppCore.doxytag

clean-local:
	rm -rf html

install-data-local: install-doxygen-html
uninstall-local: uninstall-doxygen-html

install-doxygen-html: html-local
	$(mkdir_p) "$(DESTDIR)$(htmldir)/html"
	cp -rf html/* "$(DESTDIR)$(htmldir)/html"
	$(INSTALL_DATA) hppCore.doxytag "$(DESTDIR)$(htmldir)/html"

uninstall-doxygen-html:
	rm -rf "$(DESTDIR)$(htmldir)"
