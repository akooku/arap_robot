/**
 * @file visibility_control.h
 * @brief Visibility control macros for differential drive controller
 *
 * This header must be included by all ROS 2 headers which declare symbols
 * that need to be exported for DLL/shared library builds.
 *
 * @author Addison Sears-Collins (modified for differential drive)
 * @date March 9, 2025
 */

 #ifndef DIFFERENTIAL_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
 #define DIFFERENTIAL_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
 
 // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
 //     https://gcc.gnu.org/wiki/Visibility
 
 #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
     #define DIFFERENTIAL_DRIVE_CONTROLLER_EXPORT __attribute__((dllexport))
     #define DIFFERENTIAL_DRIVE_CONTROLLER_IMPORT __attribute__((dllimport))
   #else
     #define DIFFERENTIAL_DRIVE_CONTROLLER_EXPORT __declspec(dllexport)
     #define DIFFERENTIAL_DRIVE_CONTROLLER_IMPORT __declspec(dllimport)
   #endif
 
   #ifdef DIFFERENTIAL_DRIVE_CONTROLLER_BUILDING_DLL
     #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC DIFFERENTIAL_DRIVE_CONTROLLER_EXPORT
   #else
     #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC DIFFERENTIAL_DRIVE_CONTROLLER_IMPORT
   #endif
 
   #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC_TYPE DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   #define DIFFERENTIAL_DRIVE_CONTROLLER_LOCAL
 #else
   #define DIFFERENTIAL_DRIVE_CONTROLLER_EXPORT __attribute__((visibility("default")))
   #define DIFFERENTIAL_DRIVE_CONTROLLER_IMPORT
 
   #if __GNUC__ >= 4
     #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
     #define DIFFERENTIAL_DRIVE_CONTROLLER_LOCAL  __attribute__((visibility("hidden")))
   #else
     #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
     #define DIFFERENTIAL_DRIVE_CONTROLLER_LOCAL
   #endif
 
   #define DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC_TYPE
 #endif
 
 #endif  // DIFFERENTIAL_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_ 