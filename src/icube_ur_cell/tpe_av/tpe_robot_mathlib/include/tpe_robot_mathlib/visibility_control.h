#ifndef TPE_ROBOT_MATHLIB__VISIBILITY_CONTROL_H_
#define TPE_ROBOT_MATHLIB__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TPE_ROBOT_MATHLIB_EXPORT __attribute__ ((dllexport))
    #define TPE_ROBOT_MATHLIB_IMPORT __attribute__ ((dllimport))
  #else
    #define TPE_ROBOT_MATHLIB_EXPORT __declspec(dllexport)
    #define TPE_ROBOT_MATHLIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef TPE_ROBOT_MATHLIB_BUILDING_LIBRARY
    #define TPE_ROBOT_MATHLIB_PUBLIC TPE_ROBOT_MATHLIB_EXPORT
  #else
    #define TPE_ROBOT_MATHLIB_PUBLIC TPE_ROBOT_MATHLIB_IMPORT
  #endif
  #define TPE_ROBOT_MATHLIB_PUBLIC_TYPE TPE_ROBOT_MATHLIB_PUBLIC
  #define TPE_ROBOT_MATHLIB_LOCAL
#else
  #define TPE_ROBOT_MATHLIB_EXPORT __attribute__ ((visibility("default")))
  #define TPE_ROBOT_MATHLIB_IMPORT
  #if __GNUC__ >= 4
    #define TPE_ROBOT_MATHLIB_PUBLIC __attribute__ ((visibility("default")))
    #define TPE_ROBOT_MATHLIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TPE_ROBOT_MATHLIB_PUBLIC
    #define TPE_ROBOT_MATHLIB_LOCAL
  #endif
  #define TPE_ROBOT_MATHLIB_PUBLIC_TYPE
#endif

#endif  // TPE_ROBOT_MATHLIB__VISIBILITY_CONTROL_H_
