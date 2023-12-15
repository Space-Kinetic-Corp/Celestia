set(CELMATH_SOURCES
  distance.h
  ellipsoid.h
  frustum.cpp
  frustum.h
  geomutil.h
  intersect.h
  mathlib.h
  randutils.cpp
  randutils.h
  ray.h
  solve.h
  sphere.h
  vecgl.h
        vecmath.h
)

add_library(celmath OBJECT ${CELMATH_SOURCES})
