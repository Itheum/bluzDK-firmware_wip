/**
 ******************************************************************************
  Copyright (c) 2015 Particle Industries, Inc.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#ifndef HAL_PLATFORM_H
#define	HAL_PLATFORM_H

#ifdef	__cplusplus
extern "C" {
#endif



#if PLATFORM_ID<9
    #define HAL_PLATFORM_WIFI 1
#endif

#if PLATFORM_ID==10
#define HAL_PLATFORM_CELLULAR 1
#endif
    
#if PLATFORM_ID==103
#define HAL_PLATFORM_BLUETOOTH_LE 1
#endif



#ifndef HAL_PLATFORM_WIFI
#define HAL_PLATFORM_WIFI 0
#endif

#ifndef HAL_PLATFORM_CELLULAR
#define HAL_PLATFORM_CELLULAR 0
#endif
    
#ifndef HAL_PLATFORM_BLUETOOTH_LE
#define HAL_PLATFORM_BLUETOOTH_LE 0
#endif



#ifdef	__cplusplus
}
#endif

#endif	/* HAL_PLATFORM_H */
