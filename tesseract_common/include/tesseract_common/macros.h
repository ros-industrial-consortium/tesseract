/**
 * @file macros.h
 * @brief Common Tesseract Macros
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COMMON_MACROS_H
#define TESSERACT_COMMON_MACROS_H

// clang-format off
#if defined(__GNUC__) || defined(__clang__)

#define DEPRECATED(X) __attribute__((deprecated(X)))

#if defined(__clang__)
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH				                                                                   \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"")                                           \
     _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")		                                                   \
         _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")		                                                   \
             _Pragma("GCC diagnostic ignored \"-Winconsistent-missing-override\"")	                                   \
                 _Pragma("GCC diagnostic ignored \"-Wconversion\"")			                                               \
                     _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")		                                       \
                         _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")		                               \
                             _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")		                               \
                                 _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")
#else
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH                                                                          \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"")                                           \
      _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")                                                      \
          _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")                                                     \
              _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"")                                                 \
                  _Pragma("GCC diagnostic ignored \"-Wconversion\"")                                                   \
                      _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")                                         \
                          _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")		                               \
                              _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")		                               \
                                  _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")
#endif

#define TESSERACT_COMMON_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")

#elif defined(_MSC_VER)
#define DEPRECATED(X) __declspec(deprecated(X))
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#define TESSERACT_COMMON_IGNORE_WARNINGS_POP
#else
#pragma message("WARNING: You need to implement MACROS for this compiler")
#define DEPRECATED(X)
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#define TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#define UNUSED(x) (void)(x)

// clang-format on

#endif  // TESSERACT_COMMON_MACROS_H
