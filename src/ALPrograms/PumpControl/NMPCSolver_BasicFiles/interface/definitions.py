import numpy
import ctypes

name = "Solver_PumpPressureControl"
requires_callback = True
lib = "lib/Solver_PumpPressureControl.dll"
lib_static = "lib/Solver_PumpPressureControl_static.lib"
c_header = "include/Solver_PumpPressureControl.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 48,   1),   48),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 16,   1),   16),
 ("reinitialize"        , ""      , "Solver_PumpPressureControl_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  3,),    3)]

# Info Struct Fields
info = \
[("it", ctypes.c_int),
 ("res_eq", ctypes.c_double),
 ("rsnorm", ctypes.c_double),
 ("pobj", ctypes.c_double),
 ("solvetime", ctypes.c_double),
 ("fevalstime", ctypes.c_double),
 ("QPtime", ctypes.c_double)]