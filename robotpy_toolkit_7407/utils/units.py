from unum import Unum, units

m = units.m
s = units.s
rad = units.rad
deg = units.deg
ft = Unum.unit("ft", 0.3048 * m, "foot")
inch = Unum.unit("in", ft / 12, "inch")
