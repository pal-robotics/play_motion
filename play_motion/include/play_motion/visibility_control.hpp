/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef PLAY_MOTION__VISIBILITY_CONTROL_H_
#define PLAY_MOTION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PLAY_MOTION_EXPORT __attribute__((dllexport))
#define PLAY_MOTION_IMPORT __attribute__((dllimport))
#else
#define PLAY_MOTION_EXPORT __declspec(dllexport)
#define PLAY_MOTION_IMPORT __declspec(dllimport)
#endif
#ifdef PLAY_MOTION_BUILDING_DLL
#define PLAY_MOTION_PUBLIC PLAY_MOTION_EXPORT
#else
#define PLAY_MOTION_PUBLIC PLAY_MOTION_IMPORT
#endif
#define PLAY_MOTION_PUBLIC_TYPE PLAY_MOTION_PUBLIC
#define PLAY_MOTION_LOCAL
#else
#define PLAY_MOTION_EXPORT __attribute__((visibility("default")))
#define PLAY_MOTION_IMPORT
#if __GNUC__ >= 4
#define PLAY_MOTION_PUBLIC __attribute__((visibility("default")))
#define PLAY_MOTION_LOCAL __attribute__((visibility("hidden")))
#else
#define PLAY_MOTION_PUBLIC
#define PLAY_MOTION_LOCAL
#endif
#define PLAY_MOTION_PUBLIC_TYPE
#endif

#endif  // PLAY_MOTION__VISIBILITY_CONTROL_H_
