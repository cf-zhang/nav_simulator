#ifndef PLANNER__VISIBILITY_CONTROL_H_
#define PLANNER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PLANNER_EXPORT __attribute__ ((dllexport))
    #define PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define PLANNER_EXPORT __declspec(dllexport)
    #define PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PLANNER_BUILDING_LIBRARY
    #define PLANNER_PUBLIC PLANNER_EXPORT
  #else
    #define PLANNER_PUBLIC PLANNER_IMPORT
  #endif
  #define PLANNER_PUBLIC_TYPE PLANNER_PUBLIC
  #define PLANNER_LOCAL
#else
  #define PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PLANNER_PUBLIC
    #define PLANNER_LOCAL
  #endif
  #define PLANNER_PUBLIC_TYPE
#endif

#endif  // PLANNER__VISIBILITY_CONTROL_H_
