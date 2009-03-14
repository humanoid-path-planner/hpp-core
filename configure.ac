dnl autoconf script for hppCore libs
dnl
dnl  Research carried out within the scope of the Associated
dnl  International Laboratory: Joint Japanese-French Robotics Laboratory (JRL) 
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([hppCore],[1.7],[openrobots@laas.fr])
AC_PREREQ(2.59)

if test "$CFLAGS$CXXFLAGS" = ""; then
  setdefaultflags=yes
else
  setdefaultflags=no
fi

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

if test "$setdefaultflags" = "yes"; then
  if test "$GCC" = "yes"; then
    CFLAGS="$CFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c flags])
  fi
  if test "$GXX" = "yes"; then
    CXXFLAGS="$CXXFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c++ flags])
  fi
fi

define(kwplus_reqd, [kwsPlus >= 1.6])
AC_SUBST(KWPLUS_REQD, "kwplus_reqd")
PKG_CHECK_MODULES(KWSPLUS, kwplus_reqd)

KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST(KWSPLUS_PREFIX)
AC_SUBST(KWSPLUS_CFLAGS)
AC_SUBST(KWSPLUS_LIBS)

AC_ARG_ENABLE(body, AC_HELP_STRING([--enable-body],
	[Implement ChppBody class (see INSTALL for more information)]),,[enable_body=no])

AC_SUBST(BODY_CFLAGS)
AC_SUBST(BODY_HEADER_PATH)
AC_SUBST(HPPMODEL_REQD)
AC_SUBST(HPPMODEL_CFLAGS)
AC_SUBST(HPPMODEL_LIBS)
AC_SUBST(HPPMODEL_TAGFILE, [])
AC_SUBST(HPPMODEL_PREFIX)

BODY_CFLAGS="IMPLEMENT_BODY=0"
BODY_HEADER_PATH="hppCore"

if test x${enable_body} = xyes; then
	BODY_CFLAGS="IMPLEMENT_BODY=1"
	BODY_HEADER_PATH="hppModel"
	AC_MSG_NOTICE([using internal implementation of ChppBody])
else
	define(hppmodel_reqd, [hppModel >= 1.5])
	AC_SUBST(HPPMODEL_REQD, [", hppmodel_reqd"])
	PKG_CHECK_MODULES(HPPMODEL, hppmodel_reqd)
	AC_SUBST(HPPMODEL_CFLAGS)
	AC_SUBST(HPPMODEL_LIBS)
	HPPMODEL_DOCDIR=`$PKG_CONFIG --variable=docdir hppModel`
	AC_MSG_NOTICE([using external implementation of ChppBody])
fi

AM_CONDITIONAL(BODY, test x${enable_body} = xyes)

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppCore.pc
)
