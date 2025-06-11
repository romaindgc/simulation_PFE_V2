#ifndef TPE_AV_PLUGINS__VISIBILITY_CONTROL_H_
#define TPE_AV_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TPE_AV_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define TPE_AV_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define TPE_AV_PLUGINS_EXPORT __declspec(dllexport)
    #define TPE_AV_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TPE_AV_PLUGINS_BUILDING_LIBRARY
    #define TPE_AV_PLUGINS_PUBLIC TPE_AV_PLUGINS_EXPORT
  #else
    #define TPE_AV_PLUGINS_PUBLIC TPE_AV_PLUGINS_IMPORT
  #endif
  #define TPE_AV_PLUGINS_PUBLIC_TYPE TPE_AV_PLUGINS_PUBLIC
  #define TPE_AV_PLUGINS_LOCAL
#else
  #define TPE_AV_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define TPE_AV_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define TPE_AV_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define TPE_AV_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TPE_AV_PLUGINS_PUBLIC
    #define TPE_AV_PLUGINS_LOCAL
  #endif
  #define TPE_AV_PLUGINS_PUBLIC_TYPE
#endif

#endif  // TPE_AV_PLUGINS__VISIBILITY_CONTROL_H_
