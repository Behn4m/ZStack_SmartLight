/**************************************************************************************************
  Filename:       zcl_LIGHT_data.c
  Revised:        $Date: 2014-05-12 13:14:02 -0700 (Mon, 12 May 2014) $
  Revision:       $Revision: 38502 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_poll_control.h"
#include "zcl_electrical_measurement.h"
#include "zcl_diagnostic.h"
#include "zcl_meter_identification.h"
#include "zcl_appliance_identification.h"
#include "zcl_appliance_events_alerts.h"
#include "zcl_power_profile.h"
#include "zcl_appliance_control.h"
#include "zcl_appliance_statistics.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"

#include "zcl_Smartlight.h"

/*********************************************************************
 * CONSTANTS
 */

#define LIGHT_DEVICE_VERSION     0
#define LIGHT_FLAGS              0

#define LIGHT_HWVERSION          1
#define LIGHT_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Basic Cluster

//Mandatory:
const uint8 zclLight_ZCLVersion = LIGHT_ZCLVERSION;
const uint8 zclLight_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
//Optional:
const uint8 zclLight_HWRevision = LIGHT_HWVERSION;
const uint8 zclLight_ManufacturerName[] = { 6, 'A','s','h','i','a','n'};
const uint8 zclLight_ModelId[] = { 9, 'T','S','H','A','T','0','0','0','1'};
const uint8 zclLight_DateCode[] = { 8, '2','0','1','5','0','8','0','7'};

uint8 zclLight_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclLight_PhysicalEnvironment = 0;
uint8 zclLight_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclLight_DimmableLight_IdentifyTime = 0;
#ifdef ZCL_EZMODE
uint8  zclLight_DimmableLight_IdentifyCommissionState;
#endif

uint16 zclLight_LightSensor_IdentifyTime = 0;
#ifdef ZCL_EZMODE
uint8  zclLight_LightSensor_IdentifyCommissionState;
#endif

uint16 zclLight_OccupancySensor_IdentifyTime = 0;
#ifdef ZCL_EZMODE
uint8  zclLight_OccupancySensor_IdentifyCommissionState;
#endif

// On/Off Cluster
uint8  zclLight_DimmableLight_OnOff = LIGHT_OFF;

// Level Control Cluster
#ifdef ZCL_LEVEL_CTRL
uint8  zclLight_DimmableLight_LevelCurrentLevel = ATTR_LEVEL_MIN_LEVEL;
uint16 zclLight_DimmableLight_LevelRemainingTime;
uint16 zclLight_DimmableLight_LevelOnOffTransitionTime = 20;
uint8  zclLight_DimmableLight_LevelOnLevel = ATTR_LEVEL_MID_LEVEL;
uint16 zclLight_DimmableLight_LevelOnTransitionTime = 20;
uint16 zclLight_DimmableLight_LevelOffTransitionTime = 20;
uint8  zclLight_DimmableLight_LevelDefaultMoveRate = 0;   // as fast as possible
#endif

// Groups Attributes
uint8 zclLight_DimmableLight_GroupNameSupport;

//Scenes Attributes
uint8 zclLight_DimmableLight_SceneCount;
uint8 zclLight_DimmableLight_CurrentScene;
uint16 zclLight_DimmableLight_CurrentGroup;
bool zclLight_DimmableLight_SceneValid;
uint8 zclLight_DimmableLight_ScenesNameSupport;

//Illuminance Measurment Attributes
uint16 zclLight_LightSensor_IlluminanceMeasuredValue = 0;
uint16 zclLight_LightSensor_IlluminanceMinMeasuredValue;
uint16 zclLight_LightSensor_IlluminanceMaxMeasuredValue;

//Occupancy Sensing Attributes
uint8 zclLight_OccupancySensor_Occupancy;
const uint8 zclLight_OccupancySensor_OccupancySensorType = MS_OCCUPANCY_SENSOR_TYPE_PIR;
//Attributes of the PIR Configuration Attribute Set
uint16 zclOccupancySensor_OtoUDelay = 7;
uint16 zclOccupancySensor_UtoODelay = 3;
uint8 zclOccupancySensor_UtoOThresh = 2 ;
/******************************************************************************/

#if ZCL_DISCOVER
CONST zclCommandRec_t zclLight_DimmableLight_Cmds[] =
{
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    COMMAND_BASIC_RESET_FACT_DEFAULT,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_ON,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_TOGGLE,
    CMD_DIR_SERVER_RECEIVED
  },
#ifdef ZCL_LEVEL_CONTROL
  ,{
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  }
#endif // ZCL_LEVEL_CONTROL
};

CONST uint8 zclCmdsArraySize = ( sizeof(zclLight_DimmableLight_Cmds) / sizeof(zclLight_DimmableLight_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclLight_DimmableLight_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclLight_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclLight_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_DimmableLight_IdentifyTime
    }
  },
 #ifdef ZCL_EZMODE
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_COMMISSION_STATE,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_DimmableLight_IdentifyCommissionState
    }
  },
 #endif // ZCL_EZMODE
#endif

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_DimmableLight_OnOff
    }
  }

#ifdef ZCL_LEVEL_CTRL
  , {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_CURRENT_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_DimmableLight_LevelCurrentLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_REMAINING_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_DimmableLight_LevelRemainingTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclLight_DimmableLight_LevelOnOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclLight_DimmableLight_LevelOnLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclLight_DimmableLight_LevelOnTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclLight_DimmableLight_LevelOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_DEFAULT_MOVE_RATE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclLight_DimmableLight_LevelDefaultMoveRate
    }
  }
#endif

};

uint8 CONST zclLight_DimmableLight_NumAttributes = ( sizeof(zclLight_DimmableLight_Attrs) / sizeof(zclLight_DimmableLight_Attrs[0]) );

/******************************************************************************/
CONST zclAttrRec_t zclLight_LightSensor_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclLight_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclLight_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_DimmableLight_IdentifyTime
    }
  },
 #ifdef ZCL_EZMODE
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_COMMISSION_STATE,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_DimmableLight_IdentifyCommissionState
    }
  },
 #endif // ZCL_EZMODE
#endif
  
  //*** Illuminance Measurement Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ILLUMINANCE_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_LightSensor_IlluminanceMeasuredValue
    }
  }, 
  
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ILLUMINANCE_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_LightSensor_IlluminanceMinMeasuredValue
    }
  },  

  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ILLUMINANCE_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_LightSensor_IlluminanceMaxMeasuredValue
    }
  },    

};

uint8 CONST zclLight_LightSensor_NumAttributes = ( sizeof(zclLight_LightSensor_Attrs) / sizeof(zclLight_LightSensor_Attrs[0]) );

/******************************************************************************/
CONST zclAttrRec_t zclLight_OccupancySensor_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclLight_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclLight_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclLight_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclLight_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclLight_OccupancySensor_IdentifyTime
    }
  },
 #ifdef ZCL_EZMODE
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_COMMISSION_STATE,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_OccupancySensor_IdentifyCommissionState
    }
  },
 #endif // ZCL_EZMODE
#endif
  
  //***Occupancy Sensing Cluster Attributes***
  {
    ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY,
      ZCL_DATATYPE_BITMAP8,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_OccupancySensor_Occupancy
    }
  },

  {
    ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY_SENSOR_TYPE,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ),
      (void *)&zclLight_OccupancySensor_OccupancySensorType
    }
  },   
  
};

uint8 CONST zclLight_OccupancySensor_NumAttributes = ( sizeof(zclLight_OccupancySensor_Attrs) / sizeof(zclLight_OccupancySensor_Attrs[0]) );

/*********************************************************************
 * Dimmable Light SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclLight_DimmableLight_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_GROUPS,
  ZCL_CLUSTER_ID_GEN_SCENES,
  ZCL_CLUSTER_ID_GEN_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
// work-around for compiler bug... IAR can't calculate size of array with #if options.
#ifdef ZCL_LEVEL_CTRL
 #define zclLight_DimmableLight_MAX_INCLUSTERS   6
#else
 #define zclLight_DimmableLight_MAX_INCLUSTERS   5
#endif

const cId_t zclLight_DimmableLight_OutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC
};
#define zclLight_DimmableLight_MAX_OUTCLUSTERS  (sizeof(zclLight_DimmableLight_OutClusterList) / sizeof(zclLight_DimmableLight_OutClusterList[0]))

SimpleDescriptionFormat_t zclLight_DimmableLight_SimpleDesc =
{
  DimmableLight_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
#ifdef ZCL_LEVEL_CTRL
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,        //  uint16 AppDeviceId;
#else
  ZCL_HA_DEVICEID_ON_OFF_LIGHT,          //  uint16 AppDeviceId;
#endif
  LIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  LIGHT_FLAGS,                     //  int   AppFlags:4;
  zclLight_DimmableLight_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclLight_DimmableLight_InClusterList, //  byte *pAppInClusterList;
  zclLight_DimmableLight_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclLight_DimmableLight_OutClusterList //  byte *pAppInClusterList;
};
/*********************************************************************
 * Light Sensor SIMPLE DESCRIPTOR
 */
const cId_t zclLight_LightSensor_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT
};
// work-around for compiler bug... IAR can't calculate size of array with #if options.
 #define zclLight_LightSensor_MAX_INCLUSTERS   3


const cId_t zclLight_LightSensor_OutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};
#define zclLight_LightSensor_MAX_OUTCLUSTERS  (sizeof(zclLight_LightSensor_OutClusterList) / sizeof(zclLight_LightSensor_OutClusterList[0]))

SimpleDescriptionFormat_t zclLight_LightSensor_SimpleDesc =
{
  LightSensor_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
  ZCL_HA_DEVICEID_LIGHT_SENSOR,        //  uint16 AppDeviceId;
  LIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  LIGHT_FLAGS,                     //  int   AppFlags:4;
  zclLight_LightSensor_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclLight_LightSensor_InClusterList, //  byte *pAppInClusterList;
  zclLight_LightSensor_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclLight_LightSensor_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * Occupancy Sensor SIMPLE DESCRIPTOR
 */
const cId_t zclLight_OccupancySensor_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING
};
// work-around for compiler bug... IAR can't calculate size of array with #if options.
 #define zclLight_OccupancySensor_MAX_INCLUSTERS   3

const cId_t zclLight_OccupancySensor_OutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY    
};
#define zclLight_OccupancySensor_MAX_OUTCLUSTERS  (sizeof(zclLight_OccupancySensor_OutClusterList) / sizeof(zclLight_OccupancySensor_OutClusterList[0]))

SimpleDescriptionFormat_t zclLight_OccupancySensor_SimpleDesc =
{
  OccupancySensor_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
  ZCL_HA_DEVICEID_OCCUPANCY_SENSOR,        //  uint16 AppDeviceId;
  LIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  LIGHT_FLAGS,                     //  int   AppFlags:4;
  zclLight_OccupancySensor_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclLight_OccupancySensor_InClusterList, //  byte *pAppInClusterList;
  zclLight_OccupancySensor_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclLight_OccupancySensor_OutClusterList //  byte *pAppInClusterList;
};
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/


