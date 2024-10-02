#ifndef SPEED_METER_PLUGIN__VISIBILITY_CONTROL_H_
#define SPEED_METER_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPEED_METER_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define SPEED_METER_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define SPEED_METER_PLUGIN_EXPORT __declspec(dllexport)
    #define SPEED_METER_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPEED_METER_PLUGIN_BUILDING_LIBRARY
    #define SPEED_METER_PLUGIN_PUBLIC SPEED_METER_PLUGIN_EXPORT
  #else
    #define SPEED_METER_PLUGIN_PUBLIC SPEED_METER_PLUGIN_IMPORT
  #endif
  #define SPEED_METER_PLUGIN_PUBLIC_TYPE SPEED_METER_PLUGIN_PUBLIC
  #define SPEED_METER_PLUGIN_LOCAL
#else
  #define SPEED_METER_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define SPEED_METER_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define SPEED_METER_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define SPEED_METER_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPEED_METER_PLUGIN_PUBLIC
    #define SPEED_METER_PLUGIN_LOCAL
  #endif
  #define SPEED_METER_PLUGIN_PUBLIC_TYPE
#endif

#endif  // SPEED_METER_PLUGIN__VISIBILITY_CONTROL_H_
