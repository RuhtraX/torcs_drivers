##############################################################################
#
#    file                 : Makefile
#    created              : qui mai 14 15:54:05 -03 2020
#    copyright            : (C) 2002 Arthur Cervantes
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = katerina
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car1-stock1.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-katerina_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-katerina_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
