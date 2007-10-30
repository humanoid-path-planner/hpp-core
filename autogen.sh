#!/bin/sh
aclocal
libtoolize -c
automake -ca
autoconf
