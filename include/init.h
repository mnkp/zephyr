
/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _INIT_H_
#define _INIT_H_

#include <device.h>
#include <toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System initialization levels. The PRIMARY and SECONDARY levels are
 * executed in the kernel's initialization context, which uses the interrupt
 * stack. The remaining levels are executed in the kernel's main task
 * (i.e. the nanokernel's background task or the microkernel's idle task).
 */

#define _SYS_INIT_LEVEL_PRIMARY      0
#define _SYS_INIT_LEVEL_SECONDARY    1
#define _SYS_INIT_LEVEL_NANOKERNEL   2
#define _SYS_INIT_LEVEL_MICROKERNEL  3
#define _SYS_INIT_LEVEL_APPLICATION  4


/** @def DEVICE_DEFINE
 *
 *  @brief Define device object
 *
 *  @details This macro defines a device object that is automatically
 *  configured by the kernel during system initialization.
 *
 *  @param name Device name.
 *
 *  @param data Pointer to the device's configuration data.
 *  @sa DEVICE_INIT_CONFIG_DEFINE()
 *
 *  @param level The initialization level at which configuration occurs.
 *  Must be one of the following symbols, which are listed in the order
 *  they are performed by the kernel:
 *
 *  PRIMARY: Used for devices that have no dependencies, such as those
 *  that rely solely on hardware present in the processor/SOC. These devices
 *  cannot use any kernel services during configuration, since they are not
 *  yet available.
 *
 *  SECONDARY: Used for devices that rely on the initialization of devices
 *  initialized as part of the PRIMARY level. These devices cannot use any
 *  kernel services during configuration, since they are not yet available.
 *
 *  NANOKERNEL: Used for devices that require nanokernel services during
 *  configuration.
 *
 *  MICROKERNEL: Used for devices that require microkernel services during
 *  configuration.
 *
 *  APPLICATION: Used for application components (i.e. non-kernel components)
 *  that need automatic configuration. These devices can use all services
 *  provided by the kernel during configuration.
 *
 *  @param priority The initialization priority of the device, relative to
 *  other devices of the same initialization level. Specified as an integer
 *  value in the range 0 to 99; lower values indicate earlier initialization.
 * Must be a decimal integer literal without leading zeroes or sign (e.g. 32),
 * or an equivalent symbolic name (e.g. #define MY_INIT_PRIO 32); symbolic
 * expressions are *not* permitted
 * (e.g. CONFIG_KERNEL_INIT_PRIORITY_DEFAULT + 5).
 */

#define DEVICE_DEFINE(name, data, level, priority)			    \
	 static struct device (__initconfig_##name) __used  \
	 __attribute__((__section__(".init_" #level STRINGIFY(priority)))) = { \
		 .config = &(config_##name),\
		 .driver_data = data}

/**
 * @def DEVICE_NAME_GET
 *
 * @brief Expands to the full name of a global device object
 *
 * @details Return the full name of a device object symbol created by
 * DEVICE_DEFINE(), using the @name provided to DEVICE_DEFINE().
 *
 * It is meant to be used for declaring extern symbols pointing on device
 * objects before using the DEVICE_GET macro to get the device object.
 *
 * @param name The same name provided to DEVICE_DEFINE()
 *
 * @return The exanded name of the device object created by DEVICE_DEFINE()
 */
#define DEVICE_NAME_GET(name) (_CONCAT(__initconfig_, name))

/**
 * @def DEVICE_GET
 *
 * @brief Obtain a pointer to a device object by name
 *
 * @details Return the address of a device object created by
 * DEVICE_DEFINE(), using the @name provided to DEVICE_DEFINE().
 *
 * @param name The same name provided to DEVICE_DEFINE()
 *
 * @return A pointer to the device object created by DEVICE_DEFINE()
 */
#define DEVICE_GET(name) (&DEVICE_NAME_GET(name))

#ifdef __cplusplus
}
#endif

#endif /* _INIT_H_ */
