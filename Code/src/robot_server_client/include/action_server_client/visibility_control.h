#ifndef ROBOT_SERVER_CLIENT__VISIBILITY_CONTROL_H_
#define ROBOT_SERVER_CLIENT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_SERVER_CLIENT_EXPORT __attribute__ ((dllexport))
    #define ROBOT_SERVER_CLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_SERVER_CLIENT_EXPORT __declspec(dllexport)
    #define ROBOT_SERVER_CLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_SERVER_CLIENT_BUILDING_DLL
    #define ROBOT_SERVER_CLIENT_PUBLIC ROBOT_SERVER_CLIENT_EXPORT
  #else
    #define ROBOT_SERVER_CLIENT_PUBLIC ROBOT_SERVER_CLIENT_IMPORT
  #endif
  #define ROBOT_SERVER_CLIENT_PUBLIC_TYPE ROBOT_SERVER_CLIENT_PUBLIC
  #define ROBOT_SERVER_CLIENT_LOCAL
#else
  #define ROBOT_SERVER_CLIENT_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_SERVER_CLIENT_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_SERVER_CLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_SERVER_CLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_SERVER_CLIENT_PUBLIC
    #define ROBOT_SERVER_CLIENT_LOCAL
  #endif
  #define ROBOT_SERVER_CLIENT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_SERVER_CLIENT__VISIBILITY_CONTROL_H_