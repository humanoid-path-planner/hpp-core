dnl autoconf script for hppCore libs
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([hppCore],[1.2],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

define(kwplus_reqd, [kwsPlus >= 1.1])
AC_SUBST(KWPLUS_REQD, "kwplus_reqd")
PKG_CHECK_MODULES(KWSPLUS, kwplus_reqd)

KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST(KWSPLUS_PREFIX)
AC_SUBST(KWSPLUS_CFLAGS)
AC_SUBST(KWSPLUS_LIBS)
AC_ARG_ENABLE(body, AC_HELP_STRING([--enable-body],
	[Implement ChppBody class (see INSTALL for more information)]), body=no, body=no)

AC_SUBST(BODY_CFLAGS)
AC_SUBST(BODY_HEADER_PATH)
AC_SUBST(HPPMODEL_REQD)
AC_SUBST(HPPMODEL_CFLAGS)
AC_SUBST(HPPMODEL_LIBS)
AC_SUBST(HPPMODEL_TAGFILE, [])
AC_SUBST(HPPMODEL_PREFIX)

if test x${enable_body} = xyes; then
	body=yes
fi

BODY_CFLAGS="IMPLEMENT_BODY=0"
BODY_HEADER_PATH="hppCore"

if test x${body} = xyes; then
	BODY_CFLAGS="IMPLEMENT_BODY=1"
	BODY_HEADER_PATH="hppModel"
else
	PKG_CHECK_MODULES(HPPMODEL, hppModel)
	AC_SUBST(HPPMODEL_CFLAGS)
	AC_SUBST(HPPMODEL_LIBS)
	HPPMODEL_PREFIX=`$PKG_CONFIG --variable=prefix hppModel`
	HPPMODEL_TAGFILE="$HPPMODEL_PREFIX/share/doc/doxytag/hppModel.doxytag=$HPPMODEL_PREFIX/share/doc/hppModel"
	HPPMODEL_REQD=",hppModel"
fi

AM_CONDITIONAL(BODY, test x${body} = xyes)

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppCore.pc
)
