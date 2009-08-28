# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.59])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([hppCore], [1.10.0.99], [openrobots@laas.fr], [hppCore])

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

# Automake.
AM_INIT_AUTOMAKE([1.9.6 dist-bzip2 -Wall nostdinc])

## ------------- ##
## C++ compiler  ##
## ------------- ##

# Search for a C++ compiler.
AC_LANG([C++])
AC_PROG_CXX

# Speed-up compilation.
if test "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS -pipe"
fi

# Macros are stored in build-aux.
AC_CONFIG_MACRO_DIR([build-aux])

## ------------------- ##
## Additionnal tools.  ##
## ------------------- ##

# Search for misc. common tools.
AC_PROG_MKDIR_P
AC_PROG_SED
AC_PROG_INSTALL

# Customize warnings.
JRL_CXX_WARNINGS([[-Wall],
                   [-Wcast-align],
                   [-Wcast-qual],
                   [-Wformat],
                   [-Wmissing-prototypes],
                   [-Wstrict-prototypes],
                   [-Wwrite-strings],
                   [-Wbad-function-cast],
                   [-Wmissing-declarations],
                   [-Wnested-externs]])

# Search for Libtool.
LT_INIT([dlopen])

# Search for Doxygen.
JRL_PROG_DOXYGEN

## ------------------------- ##
## Search for dependencies.  ##
## ------------------------- ##

define(kwplus_reqd, [kwsPlus >= 1.7])
AC_SUBST([KWPLUS_REQD], ["kwplus_reqd"])
PKG_CHECK_MODULES([KWSPLUS], [kwplus_reqd])

KWSPLUS_DOCDIR=`$PKG_CONFIG kwsPlus --variable=docdir`
AC_SUBST([KWSPLUS_DOCDIR])
AC_SUBST([KWSPLUS_CFLAGS])
AC_SUBST([KWSPLUS_LIBS])

AC_ARG_ENABLE(body, AC_HELP_STRING([--enable-body],
	[Implement ChppBody class (see INSTALL for more information)]),,[enable_body=no])

AC_SUBST([BODY_CFLAGS])
AC_SUBST([BODY_HEADER_PATH])
AC_SUBST([HPPMODEL_REQD])
AC_SUBST([HPPMODEL_CFLAGS])
AC_SUBST([HPPMODEL_LIBS])
AC_SUBST([HPPMODEL_TAGFILE], [])
AC_SUBST([HPPMODEL_PREFIX])

BODY_CFLAGS="IMPLEMENT_BODY=0"
BODY_HEADER_PATH="hppCore"

if test x${enable_body} = xyes; then
	BODY_CFLAGS="IMPLEMENT_BODY=1"
	BODY_HEADER_PATH="hppModel"
	AC_MSG_NOTICE([using internal implementation of ChppBody])
else
	define(hppmodel_reqd, [hppModel >= 1.6])
	AC_SUBST([HPPMODEL_REQD], [", hppmodel_reqd"])
	PKG_CHECK_MODULES([HPPMODEL], [hppmodel_reqd])
	AC_SUBST([HPPMODEL_CFLAGS])
	AC_SUBST([HPPMODEL_LIBS])
	AC_SUBST([HPPMODEL_DOCDIR])
	HPPMODEL_DOCDIR=`$PKG_CONFIG --variable=docdir hppModel`
	AC_MSG_NOTICE([using external implementation of ChppBody])
fi

AM_CONDITIONAL([BODY], [test x${enable_body} = xyes])


## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['HPP Core'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES],
	 ["${KWPLUS_REQD} ${HPPMODEL_REQD}"])
AC_SUBST([PKGCONFIG_CONFLICTS], [])
AC_SUBST([PKGCONFIG_LIBS], [-lhppCore])
AC_SUBST([PKGCONFIG_CFLAGS], [])

## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Creating Makefiles.
AC_OUTPUT([
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
])
