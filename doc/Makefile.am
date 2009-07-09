# Distributed files.
EXTRA_DIST = 				\
	Doxyfile.in			\
	additionalDoc/package.h 	\
	footer.html 			\
	header.html 			\
	package.css 			\
	pictures/footer.jpg		\
	pictures/archi.fig		\
	pictures/archi.png		\
	pictures/archi.pdf

# Generated files.
CLEANFILES =				\
	$(PACKAGE_TARNAME).doxytag	\
	Doxyfile			\
	config.log

.PHONY: 				\
	doc				\
	html				\
	doxygen 			\
	install-doxygen-html		\
	uninstall-doxygen-html

# Rules.
doc: html

html-local: doxygen

doxygen: Doxyfile
	$(DOXYGEN) "$<" \
	&& cp -rf $(top_srcdir)/doc/pictures/* html

# Doxygen generation rule.
Doxyfile: $(srcdir)/Doxyfile.in
	$(top_builddir)/config.status --file="$@":"$<" \
	&& sed -i -e 's/^#.*//' -e '/^$$/d' "$@"

# Clean rule.
clean-local:
	rm -rf html latex

# Install rules.
install-data-local: install-doxygen-html
uninstall-local: uninstall-doxygen-html

install-doxygen-html: html-local
	$(mkdir_p) "$(DESTDIR)$(htmldir)"
	cp -rf html "$(DESTDIR)$(htmldir)"
	$(INSTALL_DATA) $(PACKAGE_TARNAME).doxytag \
		"$(DESTDIR)$(htmldir)/html"

uninstall-doxygen-html:
	rm -rf "$(DESTDIR)$(htmldir)"
