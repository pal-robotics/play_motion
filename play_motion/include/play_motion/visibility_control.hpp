// Copyright 2021 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
