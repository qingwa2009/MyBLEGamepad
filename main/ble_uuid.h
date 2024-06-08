#ifndef _BLE_UUID_H_
#define _BLE_UUID_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * GATT Service UUIDs
 */
#define UUID_GAP_SERVICE 0x1800          // Generic Access Profile
#define UUID_GATT_SERVICE 0x1801         // Generic Attribute Profile
#define UUID_IMMEDIATE_ALERT_SERV 0x1802 // Immediate Alert
#define UUID_LINK_LOSS_SERV 0x1803       // Link Loss
#define UUID_TX_PWR_LEVEL_SERV 0x1804    // Tx Power
#define UUID_CURRENT_TIME_SERV 0x1805    // Current Time Service
#define UUID_REF_TIME_UPDATE_SERV 0x1806 // Reference Time Update Service
#define UUID_NEXT_DST_CHANGE_SERV 0x1807 // Next DST Change Service
#define UUID_GLUCOSE_SERV 0x1808         // Glucose
#define UUID_THERMOMETER_SERV 0x1809     // Health Thermometer
#define UUID_DEVINFO_SERV 0x180A         // Device Information
#define UUID_NWA_SERV 0x180B             // Network Availability
#define UUID_HEARTRATE_SERV 0x180D       // Heart Rate
#define UUID_PHONE_ALERT_STS_SERV 0x180E // Phone Alert Status Service
#define UUID_BATT_SERV 0x180F            // Battery Service
#define UUID_BLOODPRESSURE_SERV 0x1810   // Blood Pressure
#define UUID_ALERT_NOTIF_SERV 0x1811     // Alert Notification Service
#define UUID_HID_SERV 0x1812             // Human Interface Device
#define UUID_SCAN_PARAM_SERV 0x1813      // Scan Parameters
#define UUID_RSC_SERV 0x1814             // Running Speed and Cadence
#define UUID_CSC_SERV 0x1816             // Cycling Speed and Cadence
#define UUID_CYCPWR_SERV 0x1818          // Cycling Power
#define UUID_LOC_NAV_SERV 0x1819         // Location and Navigation

/**
 * GATT Declarations
 */
#define UUID_GATT_PRIMARY_SERVICE 0x2800   // Primary Service
#define UUID_GATT_SECONDARY_SERVICE 0x2801 // Secondary Service
#define UUID_GATT_INCLUDE 0x2802           // Include
#define UUID_GATT_CHARACTER 0x2803         // Characteristic

/**
 * GATT Descriptors
 */
#define UUID_GATT_CHAR_EXT_PROPS 0x2900  // Characteristic Extended Properties
#define UUID_GATT_CHAR_USER_DESC 0x2901  // Characteristic User Description
#define UUID_GATT_CLIENT_CHAR_CFG 0x2902 // Client Characteristic Configuration
#define UUID_GATT_SERV_CHAR_CFG 0x2903   // Server Characteristic Configuration
#define UUID_GATT_CHAR_FORMAT 0x2904     // Characteristic Presentation Format
#define UUID_GATT_CHAR_AGG_FORMAT 0x2905 // Characteristic Aggregate Format
#define UUID_GATT_VALID_RANGE 0x2906     // Valid Range
#define UUID_GATT_EXT_REPORT_REF 0x2907  // External Report Reference Descriptor
#define UUID_GATT_REPORT_REF 0x2908      // Report Reference Descriptor

/**
 * GATT Characteristics
 */
#define UUID_DEVICE_NAME 0x2A00       // Device Name
#define UUID_APPEARANCE 0x2A01        // Appearance
#define UUID_PERI_PRIVACY_FLAG 0x2A02 // Peripheral Privacy Flag
#define UUID_RECONNECT_ADDR 0x2A03    // Reconnection Address
#define UUID_PERI_CONN_PARAM 0x2A04   // Peripheral Preferred Connection Parameters
#define UUID_SERVICE_CHANGED 0x2A05   // Service Changed

/**
 * GATT Characteristic UUIDs
 */
#define UUID_ALERT_LEVEL 0x2A06            // Alert Level
#define UUID_TX_PWR_LEVEL 0x2A07           // Tx Power Level
#define UUID_DATE_TIME 0x2A08              // Date Time
#define UUID_DAY_OF_WEEK 0x2A09            // Day of Week
#define UUID_DAY_DATE_TIME 0x2A0A          // Day Date Time
#define UUID_EXACT_TIME_256 0x2A0C         // Exact Time 256
#define UUID_DST_OFFSET 0x2A0D             // DST Offset
#define UUID_TIME_ZONE 0x2A0E              // Time Zone
#define UUID_LOCAL_TIME_INFO 0x2A0F        // Local Time Information
#define UUID_TIME_WITH_DST 0x2A11          // Time with DST
#define UUID_TIME_ACCURACY 0x2A12          // Time Accuracy
#define UUID_TIME_SOURCE 0x2A13            // Time Source
#define UUID_REF_TIME_INFO 0x2A14          // Reference Time Information
#define UUID_TIME_UPDATE_CTRL_PT 0x2A16    // Time Update Control Point
#define UUID_TIME_UPDATE_STATE 0x2A17      // Time Update State
#define UUID_GLUCOSE_MEAS 0x2A18           // Glucose Measurement
#define UUID_BATT_LEVEL 0x2A19             // Battery Level
#define UUID_TEMP_MEAS 0x2A1C              // Temperature Measurement
#define UUID_TEMP_TYPE 0x2A1D              // Temperature Type
#define UUID_IMEDIATE_TEMP 0x2A1E          // Intermediate Temperature
#define UUID_MEAS_INTERVAL 0x2A21          // Measurement Interval
#define UUID_BOOT_KEY_INPUT 0x2A22         // Boot Keyboard Input Report
#define UUID_SYSTEM_ID 0x2A23              // System ID
#define UUID_MODEL_NUMBER 0x2A24           // Model Number String
#define UUID_SERIAL_NUMBER 0x2A25          // Serial Number String
#define UUID_FIRMWARE_REV 0x2A26           // Firmware Revision String
#define UUID_HARDWARE_REV 0x2A27           // Hardware Revision String
#define UUID_SOFTWARE_REV 0x2A28           // Software Revision String
#define UUID_MANUFACTURER_NAME 0x2A29      // Manufacturer Name String
#define UUID_IEEE_11073_CERT_DATA 0x2A2A   // IEEE 11073-20601 Regulatory Certification Data List
#define UUID_CURRENT_TIME 0x2A2B           // Current Time
#define UUID_SCAN_REFRESH 0x2A31           // Scan Refresh
#define UUID_BOOT_KEY_OUTPUT 0x2A32        // Boot Keyboard Output Report
#define UUID_BOOT_MOUSE_INPUT 0x2A33       // Boot Mouse Input Report
#define UUID_GLUCOSE_CONTEXT 0x2A34        // Glucose Measurement Context
#define UUID_BLOODPRESSURE_MEAS 0x2A35     // Blood Pressure Measurement
#define UUID_IMEDIATE_CUFF_PRESSURE 0x2A36 // Intermediate Cuff Pressure
#define UUID_HEARTRATE_MEAS 0x2A37         // Heart Rate Measurement
#define UUID_BODY_SENSOR_LOC 0x2A38        // Body Sensor Location
#define UUID_HEARTRATE_CTRL_PT 0x2A39      // Heart Rate Control Point
#define UUID_NETWORK_AVAIL 0x2A3E          // Network Availability
#define UUID_ALERT_STATUS 0x2A3F           // Alert Status
#define UUID_RINGER_CTRL_PT 0x2A40         // Ringer Control Point
#define UUID_RINGER_SETTING 0x2A41         // Ringer Setting
#define UUID_ALERT_CAT_ID_BMASK 0x2A42     // Alert Category ID Bit Mask
#define UUID_ALERT_CAT_ID 0x2A43           // Alert Category ID
#define UUID_ALERT_NOTIF_CTRL_PT 0x2A44    // Alert Notification Control Point
#define UUID_UNREAD_ALERT_STATUS 0x2A45    // Unread Alert Status
#define UUID_NEW_ALERT 0x2A46              // New Alert
#define UUID_SUP_NEW_ALERT_CAT 0x2A47      // Supported New Alert Category
#define UUID_SUP_UNREAD_ALERT_CAT 0x2A48   // Supported Unread Alert Category
#define UUID_BLOODPRESSURE_FEATURE 0x2A49  // Blood Pressure Feature
#define UUID_HID_INFORMATION 0x2A4A        // HID Information
#define UUID_REPORT_MAP 0x2A4B             // Report Map
#define UUID_HID_CTRL_PT 0x2A4C            // HID Control Point
#define UUID_REPORT 0x2A4D                 // Report
#define UUID_PROTOCOL_MODE 0x2A4E          // Protocol Mode
#define UUID_SCAN_INTERVAL_WINDOW 0x2A4F   // Scan Interval Window
#define UUID_PNP_ID 0x2A50                 // PnP ID
#define UUID_GLUCOSE_FEATURE 0x2A51        // Glucose Feature
#define UUID_RECORD_CTRL_PT 0x2A52         // Record Access Control Point
#define UUID_RSC_MEAS 0x2A53               // RSC Measurement
#define UUID_RSC_FEATURE 0x2A54            // RSC Feature
#define UUID_SC_CTRL_PT 0x2A55             // SC Control Point
#define UUID_CSC_MEAS 0x2A5B               // CSC Measurement
#define UUID_CSC_FEATURE 0x2A5C            // CSC Feature
#define UUID_SENSOR_LOC 0x2A5D             // Sensor Location
#define UUID_CYCPWR_MEAS 0x2A63            // Cycling Power Measurement
#define UUID_CYCPWR_VECTOR 0x2A64          // Cycling Power Vector
#define UUID_CYCPWR_FEATURE 0x2A65         // Cycling Power Feature
#define UUID_CYCPWR_CTRL_PT 0x2A66         // Cycling Power Control Point
#define UUID_LOC_SPEED 0x2A67              // Location and Speed
#define UUID_NAV 0x2A68                    // Navigation
#define UUID_POS_QUALITY 0x2A69            // Position Quality
#define UUID_LN_FEATURE 0x2A6A             // LN Feature
#define UUID_LN_CTRL_PT 0x2A6B             // LN Control Point

/**
 * GATT Unit UUIDs
 */
#define UUID_GATT_UNITLESS 0x2700                  // <Symbol>, <Expressed in terms of SI base units>
#define UUID_GATT_UNIT_LENGTH_METER 0x2701         // m, m
#define UUID_GATT_UNIT_MASS_KGRAM 0x2702           // kg, kg
#define UUID_GATT_UNIT_TIME_SECOND 0x2703          // s, s
#define UUID_GATT_UNIT_ELECTRIC_CURRENT_A 0x2704   // A, A
#define UUID_GATT_UNIT_THERMODYN_TEMP_K 0x2705     // K, K
#define UUID_GATT_UNIT_AMOUNT_SUBSTANCE_M 0x2706   // mol, mol
#define UUID_GATT_UNIT_LUMINOUS_INTENSITY_C 0x2707 // cd, cd

#define UUID_GATT_UNIT_AREA_SQ_MTR 0x2710           // m^2, m^2
#define UUID_GATT_UNIT_VOLUME_CUBIC_MTR 0x2711      // m^3, m^3
#define UUID_GATT_UNIT_VELOCITY_MPS 0x2712          // m/s, m s^-1
#define UUID_GATT_UNIT_ACCELERATION_MPS_SQ 0x2713   // m/s^2, m s^-2
#define UUID_GATT_UNIT_WAVENUMBER_RM 0x2714         // �, m^-1
#define UUID_GATT_UNIT_DENSITY_KGPCM 0x2715         // p, kg m^-3
#define UUID_GATT_UNIT_SURFACE_DENSITY_KGPSM 0x2716 // pA, kg m^-2
#define UUID_GATT_UNIT_SPECIFIC_VOLUME_CMPKG 0x2717 // v, m^3 kg^-1
#define UUID_GATT_UNIT_CURRENT_DENSITY_APSM 0x2718  // j, A m^-2
#define UUID_GATT_UNIT_MAG_FIELD_STRENGTH 0x2719    // H, A m
#define UUID_GATT_UNIT_AMOUNT_CONC_MPCM 0x271A      // c, mol m^-3
#define UUID_GATT_UNIT_MASS_CONC_KGPCM 0x271B       // c, kg m^-3
#define UUID_GATT_UNIT_LUMINANCE_CPSM 0x271C        // Lv, cd m^-2
#define UUID_GATT_UNIT_REFRACTIVE_INDEX 0x271D      // n, 1
#define UUID_GATT_UNIT_RELATIVE_PERMEABLILTY 0x271E // u, 1
#define UUID_GATT_UNIT_PLANE_ANGLE_RAD 0x2720       // rad, m m-1
#define UUID_GATT_UNIT_SOLID_ANGLE_STERAD 0x2721    // sr, m2 m-2
#define UUID_GATT_UNIT_FREQUENCY_HTZ 0x2722         // Hz, s-1
#define UUID_GATT_UNIT_FORCE_NEWTON 0x2723          // N, m kg s-2
#define UUID_GATT_UNIT_PRESSURE_PASCAL 0x2724       // Pa, N/m2 = m2 kg s-2
#define UUID_GATT_UNIT_ENERGY_JOULE 0x2725          // J, N m = m2 kg s-2
#define UUID_GATT_UNIT_POWER_WATT 0x2726            // W, J/s = m2 kg s-3
#define UUID_GATT_UNIT_E_CHARGE_C 0x2727            // C, sA
#define UUID_GATT_UNIT_E_POTENTIAL_DIF_V 0x2728     // V, W/A = m2 kg s-3 A-1

#define UUID_GATT_UNIT_CELSIUS_TEMP_DC 0x272F // oC, t/oC = T/K - 273.15

#define UUID_GATT_UNIT_TIME_MINUTE 0x2760        // min, 60 s
#define UUID_GATT_UNIT_TIME_HOUR 0x2761          // h, 3600 s
#define UUID_GATT_UNIT_TIME_DAY 0x2762           // d, 86400 s
#define UUID_GATT_UNIT_PLANE_ANGLE_DEGREE 0x2763 // o, (pi/180) rad
#define UUID_GATT_UNIT_PLANE_ANGLE_MINUTE 0x2764 // ', (pi/10800) rad
#define UUID_GATT_UNIT_PLANE_ANGLE_SECOND 0x2765 // '', (pi/648000) rad
#define UUID_GATT_UNIT_AREA_HECTARE 0x2766       // ha, 10^4 m^2
#define UUID_GATT_UNIT_VOLUME_LITRE 0x2767       // l, 10^-3 m^3
#define UUID_GATT_UNIT_MASS_TONNE 0x2768         // t, 10^3 kg

#define UUID_GATT_UINT_LENGTH_YARD 0x27A0          // yd, 0.9144 m
#define UUID_GATT_UNIT_LENGTH_PARSEC 0x27A1        // pc, 3.085678 � 1016 m
#define UUID_GATT_UNIT_LENGTH_INCH 0x27A2          // in, 0.0254 m
#define UUID_GATT_UNIT_LENGTH_FOOT 0x27A3          // ft, 0.3048 m
#define UUID_GATT_UNIT_LENGTH_MILE 0x27A4          // mi, 1609.347 m
#define UUID_GATT_UNIT_PRESSURE_PFPSI 0x27A5       // psi, 6.894757 � 103 Pa
#define UUID_GATT_UNIT_VELOCITY_KMPH 0x27A6        // km/h, 0.2777778 m^s-1
#define UUID_GATT_UNIT_VELOCITY_MPH 0x27A7         // mi/h, 0.44704 m^ s-1
#define UUID_GATT_UNIT_ANGULAR_VELOCITY_RPM 0x27A8 // r/min, 0.1047198 rad s-1
#define UUID_GATT_UNIT_ENERGY_GCAL 0x27A9          //
#define UUID_GATT_UNIT_ENERGY_KCAL 0x27AA          // kcal, 4190.02 J
#define UUID_GATT_UNIT_ENERGY_KWH 0x27AB           // kWh, 3600000 J
#define UUID_GATT_UNIT_THERMODYN_TEMP_DF 0x27AC    // oF, t/oF = T/K � 1.8 - 459.67
#define UUID_GATT_UNIT_PERCENTAGE 0x27AD           // %
#define UUID_GATT_UNIT_PER_MILE 0x27AE             //
#define UUID_GATT_UNIT_PERIOD_BPM 0x27AF           //
#define UUID_GATT_UNIT_E_CHARGE_AH 0x27B0          //
#define UUID_GATT_UNIT_MASS_DENSITY_MGPD 0x27B1    //
#define UUID_GATT_UNIT_MASS_DENSITY_MMPL 0x27B2    //
#define UUID_GATT_UNIT_TIME_YEAR 0x27B3            //
#define UUID_GATT_UNIT_TIME_MONTH 0x27B4           //

#ifdef __cplusplus
}
#endif

#endif
