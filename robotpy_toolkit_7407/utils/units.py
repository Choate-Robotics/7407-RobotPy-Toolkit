from robotpy_toolkit_7407.unum import Unum, units

m = units.m
ft = Unum.unit("ft", 0.3048 * m, "foot")
inch = Unum.unit("in", ft / 12, "inch")
mile = units.mile

s = units.s
ms = units.ms
minute = units.min
hour = units.h

rad = units.rad
deg = units.deg
rev = Unum.unit("rev", 360 * deg, "revolution")

