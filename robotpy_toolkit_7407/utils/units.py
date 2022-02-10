from unum import Unum, units

m = units.m
ft = Unum.unit("ft", 0.3048 * m, "foot")
inch = Unum.unit("in", ft / 12, "inch")

s = units.s
minute = units.min

rad = units.rad
deg = units.deg
rev = Unum.unit("rev", 360 * deg, "revolution")

