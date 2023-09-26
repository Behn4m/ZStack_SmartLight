//*****************************************************************************
//! @file       bsp.h
//! @brief      Board support package header file for CC2538 on SmartRF06EB.
//!
//! Revised     $Date: 2013-04-11 10:41:57 -0700 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef __BSP_H__
#define __BSP_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include "hw_types.h"
#include "hw_memmap.h"


/******************************************************************************
* DEFINES
******************************************************************************/

// Board LED defines
#define BSP_LED_BASE            GPIO_C_BASE
 
#define BSP_LED_1               GPIO_PIN_2      //!< PC2

#define BSP_LED_ALL             BSP_LED_1
                             



// Board key defines

#define BSP_KEY_BASE        GPIO_C_BASE     //!< Base
#define BSP_KEY_0           GPIO_PIN_3

//#define BSP_KEY_ALL             ( BSP_KEY_1|BSP_KEY_2)     //!< Bitmask of all keys
#define BSP_KEY_NWK_START         BSP_KEY_0


#define BSP_KEY_ALL         (BSP_KEY_NWK_START) //!< Bitmask of all keys // mine
  
  
// Board Occupancy Sensor defines  
  
#define BSP_OCCSENSOR_BASE      GPIO_B_BASE
#define BSP_OCCSENSOR           GPIO_PIN_0
  

/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_H__ */
