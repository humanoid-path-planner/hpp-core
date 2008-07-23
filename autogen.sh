#!/bin/sh
aclocal
libtoolize -c --force
automake -ca
autoconf
