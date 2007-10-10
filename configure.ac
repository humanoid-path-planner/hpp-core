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

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppCore.pc
)
