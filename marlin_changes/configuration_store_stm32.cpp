/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for configuration_store.cpp
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * configuration_store.cpp
 *
 * Settings and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored
 * in EEPROM  in the functions below, also increment the version number.
 * This makes sure that the default values are used whenever there is a
 * change to the data, to prevent wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same
 * order. If a feature is disabled, some data must still be written that,
 * when read,  either sets a Sane Default, or results in No Change to the
 * existing value.
 */

/* NOTE FOR STM32 (MALYAN_M300): Configuration information is store in
   one (1) page of flash memory. Though the page is 2K is size, we can
   only use a much smaller portion since we write the entire settings
   data structure to the flash page from a structure created temporarily
   on our (limited size) stack. ALSO, unlike the EEPROM, the flash page
   must be completely erased to change any portion of it. Consequently,
   to modify the flash, the entire data settings structure is read from
   flash, modified, and then written back the to flash. Reading from the
   flash is simpler the EEPROM, as the flash memory can be accessed
   directly. Finally, much of the code here is from the the original 
   "configuration_store.cpp" file. Though much of the original code is
   unnecessary for our board, it is kept intact here to match the behavior
   of the original code. 
*/

#define PRESERVED_FLASH_SIZE  0x400

#define SANITY_CHECK_FLASH_DATA_SIZE  0x800

// change EEPROM version if the structure changes
#define EEPROM_VERSION  "V56"
#define EEPROM_OFFSET  100

#define ANY(a, b, c) ENABLED(a) || ENABLED(b) || ENABLED(c)

#include "configuration_store.h"
#include "Marlin.h"
#include "language.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "stepper.h"
#include "parser.h"
#include "vector_3.h"

#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
#include "cardreader.h"
#endif

#if ENABLED(MESH_BED_LEVELING)
#include "mesh_bed_leveling.h"
#endif

#if HAS_TRINAMIC
#include "stepper_indirection.h"
#include "tmc_util.h"
#define TMC_GET_PWMTHRS(A,Q) _tmc_thrs(stepper##Q.microsteps(), stepper##Q.TPWMTHRS(), planner.axis_steps_per_mm[_AXIS(A)])
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
#include "ubl.h"
typedef struct {
    float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
} ubl_data_r;
#endif

#if ENABLED(FWRETRACT)
#include "fwretract.h"
#endif

#if ENABLED(PID_EXTRUSION_SCALING)
#define LPQ_LEN  thermalManager.lpq_len
#endif

#if ENABLED(BLTOUCH)
extern bool  bltouch_last_written_mode;
#endif

#pragma pack(push, 1) // No padding between variables

#define TMC_AXES  (MAX_EXTRUDERS + 6)

typedef struct PID {
    float Kp, Ki, Kd;
} PID;

typedef struct PIDC {
    float Kp, Ki, Kd, Kc;
} PIDC;

/**
 * Current EEPROM Layout
 *
 * Keep this data structure up to date so
 * EEPROM size is known at compile time!
 */
typedef struct SettingsDataStruct {

    char version[4];                                         // Vnn\0
    uint16_t crc;                                            // data checksum

    // DISTINCT_E_FACTORS
    uint8_t esteppers;                                       // NUM_AXIS_N
    uint32_t planner_max_acceleration_mm_per_s2[NUM_AXIS_N]; // M201 XYZE/ABCDE
    uint32_t planner_min_segment_time_us;                    // M205 Q
    float planner_axis_steps_per_mm[NUM_AXIS_N];             // M92 XYZE/ABCDE
    float planner_max_feedrate_mm_s[NUM_AXIS_N];             // M203 XYZE/ABCDE
    float planner_acceleration;                              // M204 P
    float planner_retract_acceleration;                      // M204 R
    float planner_travel_acceleration;                       // M204 T
    float planner_min_feedrate_mm_s;                         // M205 S
    float planner_min_travel_feedrate_mm_s;                  // M205 T
    float planner_max_jerk[NUM_AXIS];                        // M205 XYZE/ABCDE
    float planner_junction_deviation_mm;                     // M205 J
    float home_offset[XYZ];                                  // M206 XYZ
#if HOTENDS > 1
    float hotend_offset[XYZ][HOTENDS - 1];                   // M218 XYZ
#endif

    // ENABLE_LEVELING_FADE_HEIGHT
    float planner_z_fade_height;                             // M420 Zn

    // MESH_BED_LEVELING
    float mbl_z_offset;
    uint8_t mesh_num_x, mesh_num_y;  // GRID_MAX_POINTS_X,Y
#if ENABLED(MESH_BED_LEVELING)
    float mbl_z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
#else
    float mbl_z_values[3][3];
#endif
    
    // HAS_BED_PROBE
    float zprobe_zoffset;                                    // M851 Z

    // ABL_PLANAR
    matrix_3x3 planner_bed_level_matrix;

    // AUTO_BED_LEVELING_BILINEAR
    uint8_t grid_max_x, grid_max_y;  // GRID_MAX_POINTS_X,Y
    int bilinear_grid_spacing[2], bilinear_start[2];         // G29 L F
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];    // G29
#else
    float z_values[3][3];
#endif

    // AUTO_BED_LEVELING_UBL
    bool planner_leveling_active;                            // M420 S
    int8_t ubl_storage_slot;

    // BLTOUCH
    bool bltouch_last_written_mode;
    
    // DELTA / [XYZ]_DUAL_ENDSTOPS
#if ENABLED(DELTA)
    float delta_height;                                      // M666 H
    float delta_endstop_adj[ABC];                            // M666 XYZ
    float delta_radius;                                      // M665 R
    float delta_diagonal_rod;                                // M665 L
    float delta_segments_per_second;                         // M665 S
    float delta_calibration_radius;                          // M665 B
    float delta_tower_angle_trim[ABC];                       // M665 XYZ
#elif ENABLED(HANGPRINTER)
    float anchor_A_y;                                        // M665 W
    float anchor_A_z;                                        // M665 E
    float anchor_B_x;                                        // M665 R
    float anchor_B_y;                                        // M665 T
    float anchor_B_z;                                        // M665 Y
    float anchor_C_x;                                        // M665 U
    float anchor_C_y;                                        // M665 I
    float anchor_C_z;                                        // M665 O
    float anchor_D_z;                                        // M665 P
    float delta_segments_per_second;                         // M665 S
    float hangprinter_calibration_radius_placeholder;
#elif ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_DUAL_ENDSTOPS)
    float x_endstop_adj;                                     // M666 X
    float y_endstop_adj;                                     // M666 Y
    float z_endstop_adj;                                     // M666 Z
#endif

    // ULTIPANEL
    int16_t lcd_preheat_hotend_temp[2];                      // M145 S0 H
    int16_t lcd_preheat_bed_temp[2];                         // M145 S0 B
    int16_t lcd_preheat_fan_speed[2];                        // M145 S0 F

    // PIDTEMP
    PIDC hotendPID[MAX_EXTRUDERS];         // M301 En PIDC   // M303 En U
    int16_t lpq_len;                                         // M301 L

    // PIDTEMPBED
    PID bedPID;                            // M304 PID       // M303 E-1 U

    // HAS_LCD_CONTRAST
    int16_t lcd_contrast;                                    // M250 C

    // FWRETRACT
    bool autoretract_enabled;                                // M209 S
    float retract_length;                                    // M207 S
    float retract_feedrate_mm_s;                             // M207 F
    float retract_zlift;                                     // M207 Z
    float retract_recover_length;                            // M208 S
    float retract_recover_feedrate_mm_s;                     // M208 F
    float swap_retract_length;                               // M207 W
    float swap_retract_recover_length;                       // M208 W
    float swap_retract_recover_feedrate_mm_s;                // M208 R

    // !NO_VOLUMETRIC
    bool parser_volumetric_enabled;                          // M200 D
    float planner_filament_size[MAX_EXTRUDERS];              // M200 T D

    // HAS_TRINAMIC
#define TA  TMC_AXES                    // TMC_AXES
    uint16_t tmc_stepper_current[TA];   // M906 X Y Z X2 Y2 Z2 E0 E1 E2 E3 E4
    uint32_t tmc_hybrid_threshold[TA];  // M913 X Y Z X2 Y2 Z2 E0 E1 E2 E3 E4
    int16_t tmc_sgt[XYZ];               // M914 X Y Z

    // LIN_ADVANCE
    float planner_extruder_advance_K;                        // M900 K

    // HAS_MOTOR_CURRENT_PWM
    uint32_t motor_current_setting[XYZ];                     // M907 X Z E

    // CNC_COORDINATE_SYSTEMS
    float coordinate_system[MAX_COORDINATE_SYSTEMS][XYZ];    // G54-G59.3

    // SKEW_CORRECTION
    float planner_xy_skew_factor;                            // M852 I
    float planner_xz_skew_factor;                            // M852 J
    float planner_yz_skew_factor;                            // M852 K

    // ADVANCED_PAUSE_FEATURE
    float filament_change_unload_length[MAX_EXTRUDERS];      // M603 T U
    float filament_change_load_length[MAX_EXTRUDERS];        // M603 T L

#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    // SD CARD SORT OPTIONS
    uint8_t sort_alpha;                                      // M34 S(0|1)
    int8_t sort_folders;                                     // M34 F(-1|1|0)
#endif
    
} SettingsData;

typedef struct {
    SettingsData s;
#if ENABLED(AUTO_BED_LEVELING_UBL)
    ubl_data_r ubl;
#endif
} flash_data_r;

#pragma pack(pop)

#define REQUIRED_FLASH_SIZE (PRESERVED_FLASH_SIZE + sizeof(flash_data_r))
static_assert(REQUIRED_FLASH_SIZE <= SANITY_CHECK_FLASH_DATA_SIZE,
	      "flash_data_r size exceeds safe storage size");

const flash_data_r * flash = (flash_data_r *)
    ((uint8_t *) FLASHSTORE_ADDRESS + PRESERVED_FLASH_SIZE);

MarlinSettings settings;

#if 0
/* FIXME!? We really should be able to initialize the class here, BUT
we would need to change "configuration_store.h". SO, the function,
HAL_flashstore_init(), is called elsewhere (see HAL_stm32.c, HAL_setup()).
*/
MarlinSettings::MarlinSettings()
{
    HAL_flashstore_init();
}
#endif

static void __crc16(uint16_t * crc,  const SettingsData * s)
{
    // ###AO#### *!* a crc is marginally useful in our app,
    // so use a fake crc (simple pattern) instead to save
    // a little bit of program space
    // crc16(crc, ((uint8_t *) s) + 6, sizeof(*s) - 6);
    *crc = 0x55aa;
}

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
extern void refresh_bed_level();
#endif

uint16_t MarlinSettings::datasize() { return sizeof(SettingsData); }

/**
 * Post-process after Retrieve or Reset
 */

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
float new_z_fade_height;
#endif

void MarlinSettings::postprocess() {
    float oldpos[] = {
        current_position[X_AXIS],
	current_position[Y_AXIS],
	current_position[Z_AXIS]
    };

    // steps per s2 needs to be updated to agree with units per s2
    planner.reset_acceleration_rates();

    // make sure delta kinematics are updated before refreshing the
    // planner position so the stepper counts will be set correctly
#if ENABLED(DELTA)
    recalc_delta_settings();
#elif ENABLED(HANGPRINTER)
    recalc_hangprinter_settings();
#endif

#if ENABLED(PIDTEMP)
    thermalManager.update_pid();
#endif

#if DISABLED(NO_VOLUMETRICS)
    planner.calculate_volumetric_multipliers();
#else
    for (uint8_t i = COUNT(planner.e_factor); i--;)
	planner.refresh_e_factor(i);
#endif

#if HAS_HOME_OFFSET || ENABLED(DUAL_X_CARRIAGE)
    // software endstops depend on home_offset
    LOOP_XYZ(i) update_software_endstops((AxisEnum)i);
#endif

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    set_z_fade_height(new_z_fade_height, false); // false = no report
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    refresh_bed_level();
#endif

#if HAS_MOTOR_CURRENT_PWM
    stepper.refresh_motor_power();
#endif

#if ENABLED(FWRETRACT)
    fwretract.refresh_autoretract();
#endif

#if ENABLED(JUNCTION_DEVIATION) && ENABLED(LIN_ADVANCE)
    planner.recalculate_max_e_jerk();
#endif

    // refresh steps_to_mm with the reciprocal of axis_steps_per_mm and
    // init stepper.count[], planner.position[] with current_position
    planner.refresh_positioning();

    // various factors can change the current position
    if (memcmp(oldpos, current_position, sizeof(oldpos)))
	report_current_position();
}

#if ENABLED(EEPROM_SETTINGS)

#define DUMMY_PID_VALUE 3000.0f

const char version[4] = EEPROM_VERSION;

bool MarlinSettings::eeprom_error;
bool MarlinSettings::validating;

void MarlinSettings::write_data(int &pos, const uint8_t *value,
				uint16_t size,uint16_t *crc) {
    UNUSED(pos);
    UNUSED(value);
    UNUSED(size);
    UNUSED(crc);
}

void MarlinSettings::read_data(int &pos, uint8_t* value,
			       uint16_t size, uint16_t *crc,
			       const bool force) {
    UNUSED(pos);
    UNUSED(value);
    UNUSED(size);
    UNUSED(crc);
    UNUSED(force);
}

bool MarlinSettings::size_error(const uint16_t size) {
    UNUSED(size);
    return false;
}

#if ENABLED(AUTO_BED_LEVELING_UBL)
static int ubl_to_ubl_r(ubl_data_r * ubl_r) {
    *ubl_r = *((ubl_data_r *) ubl.z_values);
    return 0;
}

static int ubl_r_to_ubl(ubl_data_r * ubl_r) {
    *((ubl_data_r *) ubl.z_values) = *ubl_r;
    return 0;
}
#endif

// settings_data

static int settings_to_settings_r(SettingsData * s)
{
    // fill all with zeros
    memset(s, 0, sizeof(s));

    memcpy(s->version, EEPROM_VERSION, sizeof(s->version));
    //s->crc = 0x55AA; // not used

    s->esteppers = NUM_AXIS_N - MOV_AXIS;
    COPY(s->planner_max_acceleration_mm_per_s2,
	 planner.max_acceleration_mm_per_s2);
    COPY(s->planner_axis_steps_per_mm,
	 planner.axis_steps_per_mm);
    COPY(s->planner_max_feedrate_mm_s,
	 planner.max_feedrate_mm_s);
    s->planner_min_segment_time_us = planner.min_segment_time_us;
    s->planner_acceleration = planner.acceleration;
    s->planner_retract_acceleration = planner.retract_acceleration;
    s->planner_travel_acceleration = planner.travel_acceleration;
    s->planner_min_feedrate_mm_s = planner.min_feedrate_mm_s;
    s->planner_min_travel_feedrate_mm_s = planner.min_travel_feedrate_mm_s;
#if ENABLED(JUNCTION_DEVIATION)
#if ENABLED(HANGPRINTER)
    s->planner_max_jerk[0] = float(DEFAULT_AJERK);
    s->planner_max_jerk[1] = float(DEFAULT_BJERK);
    s->planner_max_jerk[2] = float(DEFAULT_CJERK);
    s->planner_max_jerk[3] = float(DEFAULT_DJERK);
    s->planner_max_jerk[4] = float(DEFAULT_EJERK);
#else
    s->planner_max_jerk[0] = float(DEFAULT_XJERK);
    s->planner_max_jerk[1] = float(DEFAULT_YJERK);
    s->planner_max_jerk[2] = float(DEFAULT_ZJERK);
    s->planner_max_jerk[3] = float(DEFAULT_EJERK);
#endif
    s->planner_junction_deviation_mm = planner.junction_deviation_mm;
#else
    COPY(s->planner_max_jerk, planner.max_jerk);
    s->planner_junction_deviation_mm = 0.2;
#endif

#if HAS_HOME_OFFSET
    COPY(s->home_offset, home_offset);
#else
    //ZF    memset(&s->home_offset, 0, sizeof(s->home_offset));
#endif
    
#if HOTENDS > 1
    // skip hotend 0 which must be 0
    for (uint8_t e = 1; e < HOTENDS; e++)
	LOOP_XYZ(i)
	    f.hotend_offset[i][e] = hotend_offset[i][e];
#endif
    // global leveling
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    s->planner_z_fade_height = planner.z_fade_height;
#else
    s->planner_z_fade_height = 10.0;
#endif
    // mesh bed leveling
#if ENABLED(MESH_BED_LEVELING)
#define MBL_SOZ sizeof(mbl.z_values)
#define MBL_SOZ0 sizeof(mbl.z_values[0][0])
    // compile time test that sizeof(mbl.z_values) is as expected
    static_assert(MBL_SOZ == GRID_MAX_POINTS * MBL_SOZ0, 
		  "MBL Z array is the wrong size.");
    //ZF s->mbl_z_offset = 0.0;
    s->mesh_num_x = GRID_MAX_POINTS_X;
    s->mesh_num_x = GRID_MAX_POINTS_Y;
    COPY(s->mbl_z_values, mbl.z_values);
#else
    // no mesh bed leveling, write a default mesh
    //ZF s->mbl_z_offset = 0.0;
    s->mesh_num_x = 3;
    s->mesh_num_x = 3;
    memset(s->mbl_z_values, 0, sizeof(s->mbl_z_values));
#endif
#if HAS_BED_PROBE
    s->zprobe_zoffset = zprobe_zoffset;
#else
    //ZF s->zprobe_zoffset = 0.0;
#endif
    // planar bed leveling matrix
#if ABL_PLANAR
    s->planner_bed_level_matrix = planner.bed_level_matrix;
#else
    //ZF memset(&s->planner_bed_level_matrix, 0,
    //ZF     sizeof(s->planner_bed_level_matrix));
#endif
    // bilinear auto bed leveling
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
#define BBL_SOZ sizeof(z_values)
#define BBL_SOZ0 sizeof(z_values[0][0])
    // compile time test that sizeof(z_values) is as expected
    static_assert(BBL_SOZ == GRID_MAX_POINTS * BBL_SOZ0,
		  "Bilinear Z array is the wrong size.");
    s->grid_max_x = GRID_MAX_POINTS_X;
    s->grid_max_y = GRID_MAX_POINTS_Y;
    COPY(s->bilinear_grid_spacing, bilinear_grid_spacing);
    COPY(s->bilinear_start,  bilinear_start);
    COPY(s->z_values, z_values);
#else
    // no bilinear bed leveling, write an empty 3x3 grid
    s->grid_max_x = 3;
    s->grid_max_y = 3;
    //ZF memset(s->bilinear_grid_spacing, 0, sizeof(s->bilinear_grid_spacing));
    //ZF memset(s->bilinear_start, 0, sizeof(s->bilinear_start));
    //ZF memset(s->z_values, 0, sizeof(s->z_values));
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
    s->planner_leveling_active = planner.leveling_active;
    s->ubl_storage_slot = ubl.storage_slot;
#else
    s->planner_leveling_active = false;
    s->ubl_storage_slot = -1;
#endif

    // BLTOUCH
#if ENABLED(BLTOUCH)
    s->bltouch_last_written_mode = bltouch_last_written_mode;
#else
    //ZF s->bltouch_last_written_mode = false;
#endif

    // 11 floats for DELTA / [XYZ]_DUAL_ENDSTOPS
#if ENABLED(DELTA)
    s->delta_height = delta_height;
    COPY(s->delta_endstop_adj, delta_endstop_adj);
    s->delta_radius = delta_radius;
    s->delta_diagonal_rod = delta_diagonal_rod;
    s->delta_segments_per_second = delta_segments_per_second;
    s->delta_calibration_radius = delta_calibration_radius;
    COPY(s->delta_tower_angle_trim, delta_tower_angle_trim);
#elif ENABLED(HANGPRINTER)
    s->anchor_A_y = anchor_A_y;
    s->anchor_A_z = anchor_A_z;
    s->anchor_B_x = anchor_B_x;
    s->anchor_B_y = anchor_B_y;
    s->anchor_B_z = anchor_B_z;
    s->anchor_C_x = anchor_C_x;
    s->anchor_C_y = anchor_C_y;
    s->anchor_C_z = anchor_C_z;
    s->anchor_D_z = anchor_D_z;
    s->delta_segments_per_second = delta_segments_per_second;
    s->hangprinter_calibration_radius_placeholder = 0.0;
#elif ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_DUAL_ENDSTOPS)
    // write dual endstops in X, Y, Z order. unused = 0.0
#if ENABLED(X_DUAL_ENDSTOPS)
    s->x_endstop_adj = endstops.x_endstop_adj;
#else
    //ZF s->x_endstop_adj = 0.0;
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
    s->y_endstop_adj = endstops.y_endstop_adj;
#else
    //ZF s->y_endstop_adj = 0.0;
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
    s->z_endstop_adj = endstops.y_endstop_adj;
#else
    //ZF s->z_endstop_adj = 0.0;
#endif
#endif

#if DISABLED(ULTIPANEL)
    int16_t lcd_preheat_hotend_temp[2] = { PREHEAT_1_TEMP_HOTEND,
					   PREHEAT_2_TEMP_HOTEND };
    int16_t lcd_preheat_bed_temp[2] =    { PREHEAT_1_TEMP_BED,
					   PREHEAT_2_TEMP_BED };
    int16_t lcd_preheat_fan_speed[2] =   { PREHEAT_1_FAN_SPEED,
					   PREHEAT_2_FAN_SPEED };
#endif
    COPY(s->lcd_preheat_hotend_temp, lcd_preheat_hotend_temp);
    COPY(s->lcd_preheat_bed_temp, lcd_preheat_bed_temp);
    COPY(s->lcd_preheat_fan_speed, lcd_preheat_fan_speed);

    for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
#if ENABLED(PIDTEMP)
        if (e < HOTENDS) {
	    s->hotendPID[e].Kp = PID_PARAM(Kp, e);
	    s->hotendPID[e].Ki = PID_PARAM(Ki, e);
	    s->hotendPID[e].Kd = PID_PARAM(Kd, e);
#if ENABLED(PID_EXTRUSION_SCALING)
	    s->hotendPID[e].Kc = PID_PARAM(Kc, e);
#else
	    s->hotendPID[e].Kc = 1.0; // default Kc
#endif
        }
        else
#endif // !PIDTEMP
        {
	    // when read, will not change the existing value
	    s->hotendPID[e].Kp = DUMMY_PID_VALUE;
	    //ZF s->hotendPID[e].Ki = 0.0;
	    //ZF s->hotendPID[e].Kd = 0.0;
	    //ZF s->hotendPID[e].Kc = 0.0;
        }
    }

#if DISABLED(PID_EXTRUSION_SCALING)
    s->lpq_len = 20;
#else
    s->lpq_len = LPQ_LEN;
#endif

#if DISABLED(PIDTEMPBED)
    s->bedPID.Kp = DUMMY_PID_VALUE;
    s->bedPID.Ki = DUMMY_PID_VALUE;
    s->bedPID.Kd = DUMMY_PID_VALUE;
#else
    s->bedPID.Kp = thermalManager.bedKp;
    s->bedPID.Ki = thermalManager.bedKi;
    s->bedPID.Kd = thermalManager.bedKd;
#endif

#if !HAS_LCD_CONTRAST
    s->lcd_contrast = 32;
#else
    s->lcd_contrast = lcd_contrast;
#endif

#if DISABLED(FWRETRACT)
    s->autoretract_enabled = false;
    s->retract_length = 3;
    s->retract_feedrate_mm_s = 45;
    //ZF s->retract_zlift = 0;
    //ZF s->retract_recover_length = 0;
    //ZF s->retract_recover_feedrate_mm_s = 0;
    s->swap_retract_length = 13;
    //ZF s->swap_retract_recover_length =0;
    s->swap_retract_recover_feedrate_mm_s = 8;
#else
    s->autoretract_enabled = fwretract.autoretract_enabled;
    s->retract_length = fwretract.retract_length;
    s->retract_feedrate_mm_s = fwretract.retract_feedrate_mm_s;
    s->retract_zlift = fwretract.retract_zlift;
    s->retract_recover_length = fwretract.retract_recover_length;
    s->retract_recover_feedrate_mm_s =
	fwretract.retract_recover_feedrate_mm_s;
    s->swap_retract_length = fwretract.swap_retract_length;
    s->swap_retract_recover_length = fwretract.swap_retract_recover_length;
    s->swap_retract_recover_feedrate_mm_s =
	fwretract.swap_retract_recover_feedrate_mm_s;
#endif

    // volumetric & filament Size
#if DISABLED(NO_VOLUMETRICS)
    s->parser_volumetric_enabled = parser.volumetric_enabled;
    // save filament sizes
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
	if (q < COUNT(planner.filament_size))
	    s->planner_filament_size[q] = planner.filament_size[q];
	//ZF else
	//ZF     s->planner_filament_size[q] = 0.0;
    }
#else
    //ZF s->parser_volumetric_enabled = false;
    //ZF for (uint8_t q = 0; q < MAX_EXTRUDERS; q++)
    //ZF     s->planner_filament_size[q] = 0.0;
#endif

    // save TMC2130 or TMC2208 configuration, and placeholder values
    uint16_t tmc_stepper_current[TMC_AXES] = {
#if HAS_TRINAMIC
#if AXIS_IS_TMC(X)
      stepperX.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(Y)
      stepperY.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(Z)
      stepperZ.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(X2)
      stepperX2.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(Y2)
      stepperY2.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(Z2)
      stepperZ2.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(E0)
      stepperE0.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(E1)
      stepperE1.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(E2)
      stepperE2.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(E3)
      stepperE3.getCurrent(),
#else
      0,
#endif
#if AXIS_IS_TMC(E4)
      stepperE4.getCurrent()
#else
      0
#endif
#else
      0
#endif
    };
    COPY(s->tmc_stepper_current, tmc_stepper_current);

    // save TMC2130 or TMC2208 hybrid threshold, and placeholder values
    uint32_t tmc_hybrid_threshold[TMC_AXES] = {
#if ENABLED(HYBRID_THRESHOLD)
#if AXIS_HAS_STEALTHCHOP(X)
      TMC_GET_PWMTHRS(X, X),
#else
      X_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(Y)
      TMC_GET_PWMTHRS(Y, Y),
#else
      Y_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(Z)
      TMC_GET_PWMTHRS(Z, Z),
#else
      Z_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(X2)
      TMC_GET_PWMTHRS(X, X2),
#else
      X2_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(Y2)
      TMC_GET_PWMTHRS(Y, Y2),
#else
      Y2_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(Z2)
      TMC_GET_PWMTHRS(Z, Z2),
#else
      Z2_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(E0)
      TMC_GET_PWMTHRS(E, E0),
#else
      E0_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(E1)
      TMC_GET_PWMTHRS(E, E1),
#else
      E1_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(E2)
      TMC_GET_PWMTHRS(E, E2),
#else
      E2_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(E3)
      TMC_GET_PWMTHRS(E, E3),
#else
      E3_HYBRID_THRESHOLD,
#endif
#if AXIS_HAS_STEALTHCHOP(E4)
      TMC_GET_PWMTHRS(E, E4)
#else
      E4_HYBRID_THRESHOLD
#endif
#else
      100, 100, 3,          // X, Y, Z
      100, 100, 3,          // X2, Y2, Z2
      30, 30, 30, 30, 30    // E0, E1, E2, E3, E4
#endif
    };
    COPY(s->tmc_hybrid_threshold, tmc_hybrid_threshold);

    // TMC2130 sensorless homing threshold
    int16_t tmc_sgt[XYZ] = {
#if ENABLED(SENSORLESS_HOMING)
#if X_SENSORLESS
      stepperX.sgt(),
#else
      0,
#endif
#if Y_SENSORLESS
      stepperY.sgt(),
#else
      0,
#endif
#if Z_SENSORLESS
      stepperZ.sgt()
#else
      0
#endif
#else
      0
#endif
    };
    COPY(s->tmc_sgt, tmc_sgt);

    // linear advance
#if ENABLED(LIN_ADVANCE)
    s->planner_extruder_advance_K = planner.extruder_advance_K;
#else
    //ZF s->planner_extruder_advance_K = 0.0;
#endif

#if HAS_MOTOR_CURRENT_PWM
    for (uint8_t q = XYZ; q--;)
	s->motor_current_setting = stepper.motor_current_setting[q];
#else
    //ZF for (uint8_t q = XYZ; q--;)
    //ZF     s->motor_current_setting[q] = 0;
#endif

      // CNC coordinate systems
#if ENABLED(CNC_COORDINATE_SYSTEMS)
    COPY(s->coordinate_system, coordinate_system);
#else
    //ZF memset(& s->coordinate_system, 0,
    //ZF     sizeof(s->coordinate_system));
#endif

    // skew correction factors
#if ENABLED(SKEW_CORRECTION)
    s->planner_xy_skew_factor = planner.xy_skew_factor;
    s->planner_xz_skew_factor = planner.xz_skew_factor;
    s->planner_yz_skew_factor = planner.yz_skew_factor;
#else
    //ZF s->planner_xy_skew_factor = 0.0;
    //ZF s->planner_xz_skew_factor = 0.0;
    //ZF s->planner_yz_skew_factor = 0.0;
#endif

    // advanced Pause filament load & unload lengths
#if ENABLED(ADVANCED_PAUSE_FEATURE)
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
        if (q < COUNT(filament_change_unload_length))
	    s->filament_change_unload_length[q] =
		filament_change_unload_length[q];
        //ZF else
	//ZF     s->filament_change_unload_length[q] = 0.0;
    }
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
        if (q < COUNT(filament_change_load_length))
	    s->filament_change_load_length[q] =
		filament_change_load_length[q];
	//ZF else
	//ZF     s->filament_change_load_length[q] = 0.0;
    }
#else
    //ZF for (uint8_t q = MAX_EXTRUDERS; q--;) {
    //ZF     s->filament_change_unload_length[q] = 0.0;
    //ZF     s->filament_change_load_length[q] = 0.0;
    //ZF }
#endif

#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    s->sort_alpha = card.getSortAlpha();
    s->sort_folders = card.getSortFolders();
#endif

    // compute the crc
    s->crc = 0;
    __crc16(&s->crc, s);

    return 0;
}

static int settings_r_to_settings(const SettingsData * s)
{
    // version has to match
    if (strncmp(s->version, EEPROM_VERSION, 4) != 0) {
#if ENABLED(EEPROM_CHITCHAT)
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM("EEPROM version mismatch ");
        SERIAL_ECHOPAIR("(EEPROM=", s->version);
        SERIAL_ECHOLNPGM(" Marlin=" EEPROM_VERSION ")");
#endif
	return 1;
    }
#if 1
    // let's do something a bit simpler here
    COPY(planner.max_acceleration_mm_per_s2,
	 s->planner_max_acceleration_mm_per_s2);
    COPY(planner.axis_steps_per_mm,
	 s->planner_axis_steps_per_mm);
    COPY(planner.max_feedrate_mm_s,
	 s->planner_max_feedrate_mm_s);
    planner.min_segment_time_us =
	s->planner_min_segment_time_us;
#else
    // number of esteppers may change
    // planner motion
    // get only the number of E stepper parameters previously stored
    // any steppers added later are set to their defaults
    const uint32_t def1[] = DEFAULT_MAX_ACCELERATION;
    const float def2[] = DEFAULT_AXIS_STEPS_PER_UNIT;
    const float def3[] = DEFAULT_MAX_FEEDRATE;
    uint32_t tmp1[NUM_AXIS_N];
    COPY(tmp1, s->planner_max_acceleration_mm_per_s2);
    planner.min_segment_time_us = s->planner_min_segment_time_us;
    float tmp2[NUM_AXIS_N];
    float tmp3[NUM_AXIS_N];
    COPY(tmp2, s->planner_axis_steps_per_mm);
    COPY(tmp3, s->planner_max_feedrate_mm_s);
    uint8_t steppers = s->esteppers + MOV_AXIS;
    LOOP_NUM_AXIS_N(i) {
	planner.max_acceleration_mm_per_s2[i] = (i < steppers) ? tmp1[i]
	    : def1[(i < COUNT(def1)) ? i : (COUNT(def1)-1)];
	planner.axis_steps_per_mm[i] = (i < steppers) ? tmp2[i]
	    : def2[(i < COUNT(def2)) ? i : (COUNT(def2)-1)];
	planner.max_feedrate_mm_s[i] = (i < steppers) ? tmp3[i]
	    : def3[(i < COUNT(def3)) ? i : (COUNT(def3)-1)];
    }
#endif
    planner.acceleration = s->planner_acceleration;
    planner.retract_acceleration = s->planner_retract_acceleration;
    planner.travel_acceleration = s->planner_travel_acceleration;
    planner.min_feedrate_mm_s = s->planner_min_feedrate_mm_s;
    planner.min_travel_feedrate_mm_s = s->planner_min_travel_feedrate_mm_s;
#if ENABLED(JUNCTION_DEVIATION)
    planner.junction_deviation_mm = planner_junction_deviation_mm;
#else
    COPY(planner.max_jerk, s->planner_max_jerk);
#endif
#if HAS_HOME_OFFSET
    // home offset (M206)
    COPY(home_offset, s->home_offset);
#endif
#if HOTENDS > 1
    // hotend offsets, if any
    // skip hotend 0 which must be 0
    for (uint8_t e = 1; e < HOTENDS; e++)
	LOOP_XYZ(i)
	    hotend_offset[i][e] = s->hotend_offset[i][e];
#endif
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    // global leveling
    new_z_fade_height = s->planner_z_fade_height;
#endif
#if ENABLED(MESH_BED_LEVELING)
    // mesh (manual) bed leveling
    mbl.z_offset = s->mbl_z_offset;
    if ((s->mesh_num_x == GRID_MAX_POINTS_X) &&
	(s->mesh_num_y == GRID_MAX_POINTS_Y)) {
	// EEPROM data fits the current mesh
	COPY(mbl.z_values, s->mbl_z_values);
    }
    else {
	// EEPROM data is stale
	mbl.reset();
    }
#endif
#if HAS_BED_PROBE
    zprobe_zoffset = s->zprobe_zoffset;
#endif
#if ABL_PLANAR
    // Planar Bed Leveling matrix
    planner.bed_level_matrix = s->planner_bed_level_matrix;
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    // Bilinear Auto Bed Leveling
    if ((s->grid_max_x == GRID_MAX_POINTS_X) &&
	(s->grid_max_y == GRID_MAX_POINTS_Y)) {
	set_bed_leveling_enabled(false);
	COPY(bilinear_grid_spacing, s->bilinear_grid_spacing);
	COPY(bilinear_start, s->bilinear_start);
	COPY(z_values, s->z_values);
    }
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
    // Unified Bed Leveling active state
    planner.leveling_active = s->planner_leveling_active;
    ubl.storage_slot = s->ubl_storage_slot;
#endif
#if ENABLED(BLTOUCH)
    // BLTOUCH
    bltouch_last_written_mode = s->bltouch_last_written_mode;
#endif
#if ENABLED(DELTA)
    // DELTA geometry ...
    delta_height = s->delta_height;
    delta_endstop_adj[0] = s->delta_endstop_adj[0];
    delta_endstop_adj[1] = s->delta_endstop_adj[1];
    delta_endstop_adj[2] = s->delta_endstop_adj[2];
    delta_radius = s->delta_radius;
    delta_diagonal_rod = s->delta_diagonal_rod;
    delta_segments_per_second = s->delta_segments_per_second;
    delta_calibration_radius = s->delta_calibration_radius;
    delta_tower_angle_trim[0] = s->delta_tower_angle_trim[0];
    delta_tower_angle_trim[1] = s->delta_tower_angle_trim[1];
    delta_tower_angle_trim[2] = s->delta_tower_angle_trim[2];
#elif ENABLED(HANGPRINTER)
    // ... or hang printer ...
    anchor_A_y = s->anchor_A_y;
    anchor_A_z = s->anchor_A_z;
    anchor_B_x = s->anchor_B_x;
    anchor_B_y = s->anchor_B_y;
    anchor_B_z = s->anchor_B_z;
    anchor_C_x = s->anchor_C_x;
    anchor_C_y = s->anchor_C_y;
    anchor_C_z = s->anchor_C_z;
    anchor_D_z = s->anchor_D_z;
    delta_segments_per_second = s->delta_segments_per_second;
#elif ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_DUAL_ENDSTOPS)
    // ... or Dual Endstops offsets
#if ENABLED(X_DUAL_ENDSTOPS)
    endstops.x_endstop_adj = s->x_endstop_adj;
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
    endstops.y_endstop_adj = s->y_endstop_adj;
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
    endstops.z_endstop_adj = s->z_endstop_adj;
#endif
#endif
#if ENABLED(ULTIPANEL)
    // LCD preheat settings
    lcd_preheat_hotend_temp[0] = s->lcd_preheat_hotend_temp[0];
    lcd_preheat_hotend_temp[1] = s->lcd_preheat_hotend_temp[1];
    lcd_preheat_bed_temp[0] = s->lcd_preheat_bed_temp[0];
    lcd_preheat_bed_temp[1] = s->lcd_preheat_bed_temp[1];
    lcd_preheat_fan_speed[0] = s->lcd_preheat_fan_speed[0];
    lcd_preheat_fan_speed[1] = s->lcd_preheat_fan_speed[1];
#endif
#if ENABLED(PIDTEMP)
    // hotend PID
    for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
	if (! (e < HOTENDS)) break;
	if (s->hotendPID[e].Kp != DUMMY_PID_VALUE) {
            // do not need to scale PID values as
	    // the values in EEPROM are already scaled
	    PID_PARAM(Kp, e) = s->hotendPID[e].Kp;
	    PID_PARAM(Ki, e) = s->hotendPID[e].Ki;
	    PID_PARAM(Kd, e) = s->hotendPID[e].Kd;
#if ENABLED(PID_EXTRUSION_SCALING)
	    PID_PARAM(Kc, e) = s->hotendPID[e].Kc;
#endif
	}
    }
#endif
#if ENABLED(PID_EXTRUSION_SCALING) 
    // PID extrusion scaling
    LPQ_LEN = s->lpq_len;
#endif
#if ENABLED(PIDTEMPBED)
    // heated bed PID
    if (s->bedPID.Kp != DUMMY_PID_VALUE) {
	thermalManager.bedKp = s->bedPID.Kp;
	thermalManager.bedKi = s->bedPID.Ki;
	thermalManager.bedKd = s->bedPID.Kd;
    }
#endif
#if HAS_LCD_CONTRAST
    // LCD contrast
    lcd_contrast = s->lcd_contrast;
#endif
#if ENABLED(FWRETRACT)
    // firmware retraction
    fwretract.autoretract_enabled = s->autorectract_enabled;
    fwretract.retract_length = s->retract_length;
    fwretract.retract_feedrate_mm_s = s->retract_feedrate_mm_s;
    fwretract.retract_zlift = s->retract_zlift;
    fwretract.retract_recover_length = s->retract_recover_length;
    fwretract.retract_recover_feedrate_mm_s =
	s->retract_recover_feedrate_mm_s;
    fwretract.swap_retract_length = s->swap_retract_length;
    fwretract.swap_retract_recover_length =
	s->swap_retract_recover_length;
    fwretract.swap_retract_recover_feedrate_mm_s =
	s->swap_retract_recover_feedrate_mm_s;
#endif
#if DISABLED(NO_VOLUMETRICS)
    // volumetric & filament size
    parser.volumetric_enabled = s->parser_volumetric_enabled;
    // save filament sizes
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
	if (q < COUNT(planner.filament_size))
	    planner.filament_size[q] = s->planner_filament_size[q];
    }
#endif
    reset_stepper_drivers();
#if HAS_TRINAMIC
    // TMC2130 Stepper Settings
#define SET_CURR(Q) stepper##Q.setCurrent(s->tmc_stepper_current[TMC_##Q] ? s->tmc_stepper_current[TMC_##Q] : Q##_CURRENT, R_SENSE, HOLD_MULTIPLIER)
#if AXIS_IS_TMC(X)
    SET_CURR(X);
#endif
#if AXIS_IS_TMC(Y)
    SET_CURR(Y);
#endif
#if AXIS_IS_TMC(Z)
    SET_CURR(Z);
#endif
#if AXIS_IS_TMC(X2)
    SET_CURR(X2);
#endif
#if AXIS_IS_TMC(Y2)
    SET_CURR(Y2);
#endif
#if AXIS_IS_TMC(Z2)
    SET_CURR(Z2);
#endif
#if AXIS_IS_TMC(E0)
    SET_CURR(E0);
#endif
#if AXIS_IS_TMC(E1)
    SET_CURR(E1);
#endif
#if AXIS_IS_TMC(E2)
    SET_CURR(E2);
#endif
#if AXIS_IS_TMC(E3)
    SET_CURR(E3);
#endif
#if AXIS_IS_TMC(E4)
    SET_CURR(E4);
#endif
#endif
#if ENABLED(HYBRID_THRESHOLD)
#define TMC_SET_PWMTHRS(A,Q) tmc_set_pwmthrs(stepper##Q, s->tmc_hybrid_threshold[TMC_##Q], planner.axis_steps_per_mm[_AXIS(A)])
#if AXIS_HAS_STEALTHCHOP(X)
    TMC_SET_PWMTHRS(X, X);
#endif
#if AXIS_HAS_STEALTHCHOP(Y)
    TMC_SET_PWMTHRS(Y, Y);
#endif
#if AXIS_HAS_STEALTHCHOP(Z)
    TMC_SET_PWMTHRS(Z, Z);
#endif
#if AXIS_HAS_STEALTHCHOP(X2)
    TMC_SET_PWMTHRS(X, X2);
#endif
#if AXIS_HAS_STEALTHCHOP(Y2)
    TMC_SET_PWMTHRS(Y, Y2);
#endif
#if AXIS_HAS_STEALTHCHOP(Z2)
    TMC_SET_PWMTHRS(Z, Z2);
#endif
#if AXIS_HAS_STEALTHCHOP(E0)
    TMC_SET_PWMTHRS(E, E0);
#endif
#if AXIS_HAS_STEALTHCHOP(E1)
    TMC_SET_PWMTHRS(E, E1);
#endif
#if AXIS_HAS_STEALTHCHOP(E2)
    TMC_SET_PWMTHRS(E, E2);
#endif
#if AXIS_HAS_STEALTHCHOP(E3)
    TMC_SET_PWMTHRS(E, E3);
#endif
#if AXIS_HAS_STEALTHCHOP(E4)
    TMC_SET_PWMTHRS(E, E4);
#endif
#endif
#if ENABLED(SENSORLESS_HOMING)
    // TMC2130 Sensorless homing threshold.
    // X and X2 use the same value
    // Y and Y2 use the same value
    // Z and Z2 use the same value
#ifdef X_HOMING_SENSITIVITY
#if AXIS_HAS_STALLGUARD(X)
    stepperX.sgt(s->tmc_sgt[0]);
#endif
#if AXIS_HAS_STALLGUARD(X2)
    stepperX2.sgt(s->tmc_sgt[0]);
#endif
#endif
#ifdef Y_HOMING_SENSITIVITY
#if AXIS_HAS_STALLGUARD(Y)
    stepperY.sgt(s->tmc_sgt[1]);
#endif
#if AXIS_HAS_STALLGUARD(Y2)
    stepperY2.sgt(s->tmc_sgt[1]);
#endif
#endif
#ifdef Z_HOMING_SENSITIVITY
#if AXIS_HAS_STALLGUARD(Z)
    stepperZ.sgt(s->tmc_sgt[2]);
#endif
#if AXIS_HAS_STALLGUARD(Z2)
    stepperZ2.sgt(s->tmc_sgt[2]);
#endif
#endif
#endif
#if ENABLED(LIN_ADVANCE)
    // linear advance
    planner.extruder_advance_K = s->planner_extruder_advance_K;
#endif
#if HAS_MOTOR_CURRENT_PWM
    // motor Current PWM
    for (uint8_t q = XYZ; q--;) 
	stepper.motor_current_setting[q] =
	    s->stepper_motor_current_setting[q];
#endif
#if ENABLED(CNC_COORDINATE_SYSTEMS)
    // CNC Coordinate System
    select_coordinate_system(-1);  // go back to machine space
    COPY(coordinate_system, s->coordinate_system); // 27 floats
#endif
#if ENABLED(SKEW_CORRECTION_GCODE)
    // skew correction factors
    planner.xy_skew_factor = s->planner_xy_skew_factor;
#if ENABLED(SKEW_CORRECTION_FOR_Z)
    planner.xz_skew_factor = s->planner_xz_skew_factor;
    planner.yz_skew_factor = s->planner_yz_skew_factor;
#endif
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
    // advanced pause filament load & unload lengths
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
	if (q < COUNT(filament_change_unload_length))
	    filament_change_unload_length[q] =
		s->filament_change_unload_length[q];
	if (q < COUNT(filament_change_load_length))
	    filament_change_load_length[q] =
		s->filament_change_load_length[q];
    }
#endif
    
#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    card.setSortOn((bool) s->sort_alpha);
    card.setSortFolders((int) s->sort_folders);
#endif
    return 0;
}


#if ENABLED(AUTO_BED_LEVELING_UBL)
#if ENABLED(EEPROM_CHITCHAT)
static void mesh_saved(uint8_t slot) {
    UNUSED(slot);
    SERIAL_PROTOCOLLNPAIR("Mesh saved in slot ", 0);
}

static void ubl_invalid_slot(const int s) {
    SERIAL_PROTOCOLLNPGM("?Invalid slot.");
    SERIAL_PROTOCOL(s);
    SERIAL_PROTOCOLLNPGM(" mesh slots available.");
}
#endif
#endif

#if 1  // **!**
/**
 * M500 - Store Configuration
 */
bool MarlinSettings::save(void)
{
    struct flash_r {
#if PRESERVED_FLASH_SIZE
	uint8_t x[PRESERVED_FLASH_SIZE];
#endif
	flash_data_r f;
    } ff;

    ff = *((flash_r *) FLASHSTORE_ADDRESS);
    if (settings_to_settings_r(&ff.f.s))
	return false;
#if ENABLED(AUTO_BED_LEVELING_UBL)
#if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
    if (ubl.storage_slot == 0)
	ubl_to_ubl_r(&ff.f.ubl);
#endif
#endif
    return ! HAL_flashstore_write((uint8_t *) &ff, sizeof(ff), 1);
}
#else
/**
 * M500 - Store Configuration
 */
bool MarlinSettings::save() {

    flash_data_r f;
    
    do {
	COPY(&f, (flash_data_r *) FLASHSTORE_ADDRESS);
	if (settings_to_settings_r(&f.s))
	    break;
#if ENABLED(AUTO_BED_LEVELING_UBL)
#if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
	if (ubl.storage_slot == 0)
	    if (ubl_to_ubl_r(&f.ubl))
		break;
#endif
#endif
	if (HAL_flashstore_write((uint8_t *) &f, sizeof(f), 1))
	    break;
#if ENABLED(EEPROM_CHITCHAT)
	SERIAL_ECHO_START();
	SERIAL_ECHOPAIR("Settings Stored (", (uint32_t) sizeof(f.s));
	SERIAL_ECHOPAIR(" bytes; crc ", (uint32_t) f.s.crc);
	SERIAL_ECHOLNPGM(")");
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
#if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
#if ENABLED(EEPROM_CHITCHAT)
	if (! (ubl.storage_slot < 0)) {
	    if (ubl.storage_slot == 0)
		// we've only one slot
		mesh_saved(0);
	    else
		ubl_invalid_slot(ubl.storage_slot);
	}
#endif
#endif
#endif
	return true;
    } while(0);
    return false;
}
#endif

/**
 * M501 - Retrieve Configuration
 */
bool MarlinSettings::load() {
    bool e = false;
    do {
	uint16_t crc = 0;
	__crc16(&crc, &flash->s);
	if (crc != flash->s.crc) {
#if ENABLED(EEPROM_CHITCHAT)
	    SERIAL_ERROR_START();
	    SERIAL_ERRORPGM("EEPROM CRC mismatch - (stored) ");
	    SERIAL_ERROR(flash->s.crc);
	    SERIAL_ERRORPGM(" != ");
	    SERIAL_ERROR(crc);
	    SERIAL_ERRORLNPGM(" (calculated)!");
#endif
	    reset();
	    break;
	}
	if (settings_r_to_settings(&flash->s)) {
#if ENABLED(EEPROM_CHITCHAT)
	    SERIAL_ERROR_START();
	    SERIAL_ERRORLNPGM("EEPROM version mismatch");
#endif
	    reset();
	    break;
	}
#if ENABLED(EEPROM_CHITCHAT)
	SERIAL_ECHO_START();
	SERIAL_ECHO(version);
	SERIAL_ECHOPAIR(" stored settings retrieved (",
			(uint32_t) sizeof(flash->s));
	SERIAL_ECHOPAIR(" bytes; crc ",
			(uint32_t) flash->s.crc);
	SERIAL_ECHOLNPGM(")");
#endif
	postprocess();
#if ENABLED(AUTO_BED_LEVELING_UBL)
	ubl.report_state();
#if ENABLED(EEPROM_CHITCHAT)
	SERIAL_EOL();
	ubl.echo_name();
	SERIAL_ECHOLNPGM(" initialized.");
#endif
	if (ubl.storage_slot >= 0) {
	    load_mesh(ubl.storage_slot);
#if ENABLED(EEPROM_CHITCHAT)
	    SERIAL_ECHOPAIR("Mesh ", ubl.storage_slot);
	    SERIAL_ECHOLNPGM(" loaded from storage.");
#endif
	}
	else {
            ubl.reset();
#if ENABLED(EEPROM_CHITCHAT)
	    SERIAL_ECHOLNPGM("UBL System reset()");
#endif
	}
#endif
	e = true;
    } while(0);
#if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
    report();
#endif
    return e;
}

bool MarlinSettings::_load() {
    // not used
    return true;
}

bool MarlinSettings::validate() {
    uint16_t crc = 0;
    __crc16(&crc, &flash->s);
    return (crc == flash->s.crc);
}

#if ENABLED(AUTO_BED_LEVELING_UBL)
uint16_t MarlinSettings::meshes_start_index() {
    // pad the end of configuration data so it can float up
    // or down a little bit without disrupting the mesh data
    //return (datasize() + EEPROM_OFFSET + 32) & 0xFFF8;
    return ((uint8_t *) &flash->ubl) - ((uint8_t *) flash);
}

uint16_t MarlinSettings::calc_num_meshes() {
    // return (meshes_end - meshes_start_index()) / sizeof(ubl.z_values);
    return 1;
}

int MarlinSettings::mesh_slot_offset(const int8_t slot) {
    // return meshes_end - (slot + 1) * sizeof(ubl.z_values);
    UNUSED(slot);
    return meshes_start_index();
}

void MarlinSettings::store_mesh(const int8_t slot)
{
    struct flash_r {
#if PRESERVE_FLASH
	uint32_t x[0x100];
#endif
	flash_data_r f;
    } ff;

    // !!! written for only 1 slot

    if (slot != 0) {
#if ENABLED(EEPROM_CHITCHAT)
	ubl_invalid_slot(1);
	SERIAL_PROTOCOLPAIR("E2END=", E2END);
	SERIAL_PROTOCOLPAIR(" meshes_end=", meshes_end);
	SERIAL_PROTOCOLLNPAIR(" slot=", slot);
	SERIAL_EOL();
#endif
	return;
    }

    ff = *((flash_r *) FLASHSTORE_ADDRESS);
    ff.f.ubl = *((ubl_data_r *) ubl.z_values);

    if (HAL_flashstore_write((uint8_t *) &ff, sizeof(ff), 1)) {
#if ENABLED(EEPROM_CHITCHAT)
    SERIAL_ERRORLNPGM("ERROR writing flash!");
#endif
	return;
    }

    // TODO? write crc to MAT along with other data,
    // or just tack on to the beginning or end
#if ENABLED(EEPROM_CHITCHAT)
	mesh_saved(0);
#endif
}

void MarlinSettings::load_mesh(const int8_t slot, void * const into) {

    // !!! written for only 1 slot

#if ENABLED(EEPROM_CHITCHAT)
    if (slot == 0) {
	*((ubl_data_r *) ubl.z_values) = flash->ubl;
	SERIAL_PROTOCOLLNPAIR("Mesh loaded from slot ", slot);
    }
    if (slot > 0)
	ubl_invalid_slot(1);
#else
    if (slot == 0)
	*((ubl_data_r *) ubl.z_values) = flash->ubl;
#endif
}

// void MarlinSettings::delete_mesh() {
// }
// void MarlinSettings::defrag_meshes() {
// }
#endif

#else
bool MarlinSettings::save() {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM("EEPROM disabled");
    return false;
}
#endif


/**
 * M502 - Reset Configuration
 */
void MarlinSettings::reset() {
    
    const float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
    const float tmp2[] = DEFAULT_MAX_FEEDRATE;
    const uint32_t tmp3[] = DEFAULT_MAX_ACCELERATION;

    LOOP_NUM_AXIS_N(i) {
	planner.axis_steps_per_mm[i] =
	    pgm_read_float(&tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1]);
	planner.max_feedrate_mm_s[i] =
	    pgm_read_float(&tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1]);
	planner.max_acceleration_mm_per_s2[i] =
	    pgm_read_dword_near(&tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1]);
    }

    planner.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
    planner.acceleration = DEFAULT_ACCELERATION;
    planner.retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
    planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
    planner.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
    planner.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;

#if ENABLED(JUNCTION_DEVIATION)
    planner.junction_deviation_mm = float(JUNCTION_DEVIATION_MM);
#else
#if ENABLED(HANGPRINTER)
    planner.max_jerk[A_AXIS] = DEFAULT_AJERK;
    planner.max_jerk[B_AXIS] = DEFAULT_BJERK;
    planner.max_jerk[C_AXIS] = DEFAULT_CJERK;
    planner.max_jerk[D_AXIS] = DEFAULT_DJERK;
#else
    planner.max_jerk[X_AXIS] = DEFAULT_XJERK;
    planner.max_jerk[Y_AXIS] = DEFAULT_YJERK;
    planner.max_jerk[Z_AXIS] = DEFAULT_ZJERK;
#endif
    planner.max_jerk[E_AXIS] = DEFAULT_EJERK;
#endif

#if HAS_HOME_OFFSET
    ZERO(home_offset);
#endif

#if HOTENDS > 1
    const float tmp4[XYZ][HOTENDS] = {
        HOTEND_OFFSET_X,
	HOTEND_OFFSET_Y
#if HAS_HOTEND_OFFSET_Z
	, HOTEND_OFFSET_Z
#else
	, { 0 }
#endif
    };
    
    static_assert(
      tmp4[X_AXIS][0] == 0 && tmp4[Y_AXIS][0] == 0 && tmp4[Z_AXIS][0] == 0,
      "Offsets for the first hotend must be 0.0.");

    LOOP_XYZ(i) HOTEND_LOOP() hotend_offset[i][e] = tmp4[i][e];
#endif

  // Global Leveling
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    new_z_fade_height = 0.0;
#endif
#if HAS_LEVELING
    reset_bed_level();
#endif
#if HAS_BED_PROBE
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif

#if ENABLED(DELTA)
    const float adj[ABC] = DELTA_ENDSTOP_ADJ;
    const float dta[ABC] = DELTA_TOWER_ANGLE_TRIM;
    delta_height = DELTA_HEIGHT;
    COPY(delta_endstop_adj, adj);
    delta_radius = DELTA_RADIUS;
    delta_diagonal_rod = DELTA_DIAGONAL_ROD;
    delta_segments_per_second = DELTA_SEGMENTS_PER_SECOND;
    delta_calibration_radius = DELTA_CALIBRATION_RADIUS;
    COPY(delta_tower_angle_trim, dta);
#elif ENABLED(HANGPRINTER)
    anchor_A_y = float(ANCHOR_A_Y);
    anchor_A_z = float(ANCHOR_A_Z);
    anchor_B_x = float(ANCHOR_B_X);
    anchor_B_y = float(ANCHOR_B_Y);
    anchor_B_z = float(ANCHOR_B_Z);
    anchor_C_x = float(ANCHOR_C_X);
    anchor_C_y = float(ANCHOR_C_Y);
    anchor_C_z = float(ANCHOR_C_Z);
    anchor_D_z = float(ANCHOR_D_Z);
    delta_segments_per_second = KINEMATIC_SEGMENTS_PER_SECOND;
#elif ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_DUAL_ENDSTOPS)
#if ENABLED(X_DUAL_ENDSTOPS)
    endstops.x_endstop_adj = (
#ifdef X_DUAL_ENDSTOPS_ADJUSTMENT
			      X_DUAL_ENDSTOPS_ADJUSTMENT
#else
			      0
#endif
			      );
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
    endstops.y_endstop_adj = (
#ifdef Y_DUAL_ENDSTOPS_ADJUSTMENT
			      Y_DUAL_ENDSTOPS_ADJUSTMENT
#else
			      0
#endif
			      );
#endif
    #if ENABLED(Z_DUAL_ENDSTOPS)
    endstops.z_endstop_adj = (
#ifdef Z_DUAL_ENDSTOPS_ADJUSTMENT
			      Z_DUAL_ENDSTOPS_ADJUSTMENT
#else
			      0
#endif
			      );
#endif
#endif

#if ENABLED(ULTIPANEL)
    lcd_preheat_hotend_temp[0] = PREHEAT_1_TEMP_HOTEND;
    lcd_preheat_hotend_temp[1] = PREHEAT_2_TEMP_HOTEND;
    lcd_preheat_bed_temp[0] = PREHEAT_1_TEMP_BED;
    lcd_preheat_bed_temp[1] = PREHEAT_2_TEMP_BED;
    lcd_preheat_fan_speed[0] = PREHEAT_1_FAN_SPEED;
    lcd_preheat_fan_speed[1] = PREHEAT_2_FAN_SPEED;
#endif

#if ENABLED(PIDTEMP)
#if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
    HOTEND_LOOP()
#endif
    {
	PID_PARAM(Kp, e) = float(DEFAULT_Kp);
	PID_PARAM(Ki, e) = scalePID_i(DEFAULT_Ki);
	PID_PARAM(Kd, e) = scalePID_d(DEFAULT_Kd);
#if ENABLED(PID_EXTRUSION_SCALING)
        PID_PARAM(Kc, e) = DEFAULT_Kc;
#endif
    }
#if ENABLED(PID_EXTRUSION_SCALING)
    thermalManager.lpq_len = 20; // default last-position-queue size
#endif
#endif // PIDTEMP

#if ENABLED(PIDTEMPBED)
    thermalManager.bedKp = DEFAULT_bedKp;
    thermalManager.bedKi = scalePID_i(DEFAULT_bedKi);
    thermalManager.bedKd = scalePID_d(DEFAULT_bedKd);
#endif

#if HAS_LCD_CONTRAST
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif

#if ENABLED(FWRETRACT)
    fwretract.reset();
#endif

#if DISABLED(NO_VOLUMETRICS)
    parser.volumetric_enabled =
#if ENABLED(VOLUMETRIC_DEFAULT_ON)
        true
#else
        false
#endif
	;
    for (uint8_t q = 0; q < COUNT(planner.filament_size); q++)
	planner.filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;
  #endif
    
    endstops.enable_globally(
#if ENABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
			     true
#else
			     false
#endif
			     );

    reset_stepper_drivers();
    
#if ENABLED(LIN_ADVANCE)
    planner.extruder_advance_K = LIN_ADVANCE_K;
#endif

#if HAS_MOTOR_CURRENT_PWM
    uint32_t tmp_motor_current_setting[XYZ] = PWM_MOTOR_CURRENT;
    for (uint8_t q = XYZ; q--;)
	stepper.digipot_current(q,
	  (stepper.motor_current_setting[q] = tmp_motor_current_setting[q]));
#endif
    
#if ENABLED(SKEW_CORRECTION_GCODE)
    planner.xy_skew_factor = XY_SKEW_FACTOR;
#if ENABLED(SKEW_CORRECTION_FOR_Z)
    planner.xz_skew_factor = XZ_SKEW_FACTOR;
    planner.yz_skew_factor = YZ_SKEW_FACTOR;
#endif
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
	filament_change_unload_length[e] = FILAMENT_CHANGE_UNLOAD_LENGTH;
	filament_change_load_length[e] = FILAMENT_CHANGE_FAST_LOAD_LENGTH;
    }
#endif

#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    card.setSortOn(true);
    card.setSortFolders(FOLDER_SORTING);
#endif

    postprocess();

#if ENABLED(EEPROM_CHITCHAT)
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
#endif
}

#if DISABLED(DISABLE_M503)
/**
 * M503 - Report current settings in RAM
 * define a few supporting functions here
 */

#define CONFIG_ECHO_START do{ if(!forReplay) SERIAL_ECHO_START(); }while(0)

#if HAS_TRINAMIC
void say_M906() { SERIAL_ECHOPGM("  M906"); }
#if ENABLED(HYBRID_THRESHOLD)
void say_M913() { SERIAL_ECHOPGM("  M913"); }
#endif
#if ENABLED(SENSORLESS_HOMING)
void say_M914() { SERIAL_ECHOPGM("  M914"); }
#endif
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
void say_M603() { SERIAL_ECHOPGM("  M603 "); }
#endif

inline void say_units(const bool colon=false) {
    serialprintPGM(
#if ENABLED(INCH_MODE_SUPPORT)
		   parser.linear_unit_factor != 1.0 ? PSTR(" (in)") :
#endif
		   PSTR(" (mm)")
		   );
    if (colon) SERIAL_ECHOLNPGM(":");
}

/**
 * M503 - Report current settings in RAM
 *
 * Unless specifically disabled, M503 is available even without EEPROM
 */
void MarlinSettings::report(const bool forReplay) {

    // announce current units, in case inches are being displayed
    CONFIG_ECHO_START;
#if ENABLED(INCH_MODE_SUPPORT)
#define LINEAR_UNIT(N) (float(N) / parser.linear_unit_factor)
#define VOLUMETRIC_UNIT(N) (float(N) / (parser.volumetric_enabled ? parser.volumetric_unit_factor : parser.linear_unit_factor))
    SERIAL_ECHOPGM("  G2");
    SERIAL_CHAR(parser.linear_unit_factor == 1.0 ? '1' : '0');
    SERIAL_ECHOPGM(" ;");
    say_units();
#else
#define LINEAR_UNIT(N) (N)
#define VOLUMETRIC_UNIT(N) (N)
    SERIAL_ECHOPGM("  G21    ;");
    say_units();
#endif
    SERIAL_EOL();

    // Temperature units - for Ultipanel temperature options
#if ENABLED(ULTIPANEL)
    CONFIG_ECHO_START;
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
#define TEMP_UNIT(N) parser.to_temp_units(N)
    SERIAL_ECHOPGM("  M149 ");
    SERIAL_CHAR(parser.temp_units_code());
    SERIAL_ECHOPGM(" ; Units in ");
    serialprintPGM(parser.temp_units_name());
#else
#define TEMP_UNIT(N) (N)
    SERIAL_ECHOLNPGM("  M149 C ; Units in Celsius");
#endif
    SERIAL_EOL();
#endif
    
    // Volumetric extrusion M200
#if DISABLED(NO_VOLUMETRICS)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOPGM("Filament settings:");
        if (parser.volumetric_enabled)
	    SERIAL_EOL();
        else
	    SERIAL_ECHOLNPGM(" Disabled");
    }

    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 D", LINEAR_UNIT(planner.filament_size[0]));
    SERIAL_EOL();
#if EXTRUDERS > 1
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 T1 D", LINEAR_UNIT(planner.filament_size[1]));
    SERIAL_EOL();
#if EXTRUDERS > 2
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 T2 D", LINEAR_UNIT(planner.filament_size[2]));
    SERIAL_EOL();
#if EXTRUDERS > 3
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 T3 D", LINEAR_UNIT(planner.filament_size[3]));
    SERIAL_EOL();
#if EXTRUDERS > 4
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 T4 D", LINEAR_UNIT(planner.filament_size[4]));
    SERIAL_EOL();
#endif // EXTRUDERS > 4
#endif // EXTRUDERS > 3
#endif // EXTRUDERS > 2
#endif // EXTRUDERS > 1
    if (!parser.volumetric_enabled) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("  M200 D0");
    }
#endif

    if (!forReplay) {
	CONFIG_ECHO_START;
#if ENABLED(INCH_MODE_SUPPORT)
	SERIAL_ECHOLNPGM("Steps per unit:");
#else
	SERIAL_ECHOLNPGM("Steps per mm:");
#endif
    }
    CONFIG_ECHO_START;
#if ENABLED(HANGPRINTER)
    SERIAL_ECHOPAIR("  M92 A", LINEAR_UNIT(planner.axis_steps_per_mm[A_AXIS]));
    SERIAL_ECHOPAIR(" B", LINEAR_UNIT(planner.axis_steps_per_mm[B_AXIS]));
    SERIAL_ECHOPAIR(" C", LINEAR_UNIT(planner.axis_steps_per_mm[C_AXIS]));
    SERIAL_ECHOPAIR(" D", LINEAR_UNIT(planner.axis_steps_per_mm[D_AXIS]));
#else
    SERIAL_ECHOPAIR("  M92 X", LINEAR_UNIT(planner.axis_steps_per_mm[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.axis_steps_per_mm[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.axis_steps_per_mm[Z_AXIS]));
#endif
#if DISABLED(DISTINCT_E_FACTORS)
    SERIAL_ECHOPAIR(" E", VOLUMETRIC_UNIT(planner.axis_steps_per_mm[E_AXIS]));
#endif
    SERIAL_EOL();
#if ENABLED(DISTINCT_E_FACTORS)
    CONFIG_ECHO_START;
    for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M92 T", (int)i);
        SERIAL_ECHOLNPAIR(" E",
	  VOLUMETRIC_UNIT(planner.axis_steps_per_mm[E_AXIS + i]));
    }
#endif

    if (!forReplay) {
	CONFIG_ECHO_START;
#if ENABLED(INCH_MODE_SUPPORT)
	SERIAL_ECHOLNPGM("Maximum feedrates (units/s):");
#else
	SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
#endif
    }
    CONFIG_ECHO_START;
#if ENABLED(HANGPRINTER)
    SERIAL_ECHOPAIR("  M203 A",
      LINEAR_UNIT(planner.max_feedrate_mm_s[A_AXIS]));
    SERIAL_ECHOPAIR(" B", LINEAR_UNIT(planner.max_feedrate_mm_s[B_AXIS]));
    SERIAL_ECHOPAIR(" C", LINEAR_UNIT(planner.max_feedrate_mm_s[C_AXIS]));
    SERIAL_ECHOPAIR(" D", LINEAR_UNIT(planner.max_feedrate_mm_s[D_AXIS]));
#else
    SERIAL_ECHOPAIR("  M203 X",
      LINEAR_UNIT(planner.max_feedrate_mm_s[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_feedrate_mm_s[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.max_feedrate_mm_s[Z_AXIS]));
#endif
#if DISABLED(DISTINCT_E_FACTORS)
    SERIAL_ECHOPAIR(" E", VOLUMETRIC_UNIT(planner.max_feedrate_mm_s[E_AXIS]));
#endif
    SERIAL_EOL();
#if ENABLED(DISTINCT_E_FACTORS)
    CONFIG_ECHO_START;
    for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M203 T", (int)i);
        SERIAL_ECHOLNPAIR(" E",
	  VOLUMETRIC_UNIT(planner.max_feedrate_mm_s[E_AXIS + i]));
    }
#endif

    if (!forReplay) {
	CONFIG_ECHO_START;
#if ENABLED(INCH_MODE_SUPPORT)
	SERIAL_ECHOLNPGM("Maximum Acceleration (units/s2):");
#else
	SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
#endif
    }
    CONFIG_ECHO_START;
#if ENABLED(HANGPRINTER)
    SERIAL_ECHOPAIR("  M201 A",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[A_AXIS]));
    SERIAL_ECHOPAIR(" B",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[B_AXIS]));
    SERIAL_ECHOPAIR(" C",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[C_AXIS]));
    SERIAL_ECHOPAIR(" D",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[D_AXIS]));
#else
    SERIAL_ECHOPAIR("  M201 X",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[X_AXIS]));
    SERIAL_ECHOPAIR(" Y",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z",
      LINEAR_UNIT(planner.max_acceleration_mm_per_s2[Z_AXIS]));
#endif
#if DISABLED(DISTINCT_E_FACTORS)
    SERIAL_ECHOPAIR(" E",
      VOLUMETRIC_UNIT(planner.max_acceleration_mm_per_s2[E_AXIS]));
#endif
    SERIAL_EOL();
#if ENABLED(DISTINCT_E_FACTORS)
    CONFIG_ECHO_START;
    for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M201 T", (int)i);
        SERIAL_ECHOLNPAIR(" E",
	  VOLUMETRIC_UNIT(planner.max_acceleration_mm_per_s2[E_AXIS + i]));
    }
#endif

    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM(
#if ENABLED(INCH_MODE_SUPPORT)
//"Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>"
#else
"Acceleration (mm/s2): P<print_accel> R<retract_accel> T<travel_accel>"
#endif
			 );
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M204 P", LINEAR_UNIT(planner.acceleration));
    SERIAL_ECHOPAIR(" R", LINEAR_UNIT(planner.retract_acceleration));
    SERIAL_ECHOLNPAIR(" T", LINEAR_UNIT(planner.travel_acceleration));
    
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOPGM(
#if ENABLED(INCH_MODE_SUPPORT)
"Advanced: Q<min_segment_time_us> S<min_feedrate> T<min_travel_feedrate>"
#else
"Advanced: Q<min_segment_time_us> S<min_feedrate> T<min_travel_feedrate>"
#endif
		       );
#if ENABLED(JUNCTION_DEVIATION)
        SERIAL_ECHOPGM(" J<junc_dev>");
#else
#if ENABLED(HANGPRINTER)
	SERIAL_ECHOPGM(
" A<max_a_jerk> B<max_b_jerk> C<max_c_jerk> D<max_d_jerk>"
		       );
#else
	SERIAL_ECHOPGM(" X<max_x_jerk> Y<max_y_jerk> Z<max_z_jerk>");
#endif
#endif
#if DISABLED(JUNCTION_DEVIATION) || ENABLED(LIN_ADVANCE)
        SERIAL_ECHOPGM(" E<max_e_jerk>");
#endif
	SERIAL_EOL();
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M205 Q", LINEAR_UNIT(planner.min_segment_time_us));
    SERIAL_ECHOPAIR(" S", LINEAR_UNIT(planner.min_feedrate_mm_s));
    SERIAL_ECHOPAIR(" T", LINEAR_UNIT(planner.min_travel_feedrate_mm_s));

#if ENABLED(JUNCTION_DEVIATION)
    SERIAL_ECHOPAIR(" J", LINEAR_UNIT(planner.junction_deviation_mm));
#else
#if ENABLED(HANGPRINTER)
    SERIAL_ECHOPAIR(" A", LINEAR_UNIT(planner.max_jerk[A_AXIS]));
    SERIAL_ECHOPAIR(" B", LINEAR_UNIT(planner.max_jerk[B_AXIS]));
    SERIAL_ECHOPAIR(" C", LINEAR_UNIT(planner.max_jerk[C_AXIS]));
    SERIAL_ECHOPAIR(" D", LINEAR_UNIT(planner.max_jerk[D_AXIS]));
#else
    SERIAL_ECHOPAIR(" X", LINEAR_UNIT(planner.max_jerk[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_jerk[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.max_jerk[Z_AXIS]));
#endif
    SERIAL_ECHOPAIR(" E", LINEAR_UNIT(planner.max_jerk[E_AXIS]));
#endif
    SERIAL_EOL();
    
#if HAS_M206_COMMAND
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Home offset:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X", LINEAR_UNIT(home_offset[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(home_offset[Y_AXIS]));
    SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(home_offset[Z_AXIS]));
#endif

#if HOTENDS > 1
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Hotend offsets:");
    }
    CONFIG_ECHO_START;
    for (uint8_t e = 1; e < HOTENDS; e++) {
        SERIAL_ECHOPAIR("  M218 T", (int)e);
        SERIAL_ECHOPAIR(" X", LINEAR_UNIT(hotend_offset[X_AXIS][e]));
        SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(hotend_offset[Y_AXIS][e]));
#if HAS_HOTEND_OFFSET_Z
	SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(hotend_offset[Z_AXIS][e]));
#endif
        SERIAL_EOL();
    }
#endif

    // Bed Leveling
#if HAS_LEVELING
#if ENABLED(MESH_BED_LEVELING)
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM("Mesh Bed Leveling:");
    }
#elif ENABLED(AUTO_BED_LEVELING_UBL)
    if (!forReplay) {
	CONFIG_ECHO_START;
	ubl.echo_name();
	SERIAL_ECHOLNPGM(":");
    }
#elif HAS_ABL
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM("Auto Bed Leveling:");
    }
#endif

    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M420 S", planner.leveling_active ? 1 : 0);
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.z_fade_height));
#endif
    SERIAL_EOL();
#if ENABLED(MESH_BED_LEVELING)
    if (leveling_is_valid()) {
	for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
		CONFIG_ECHO_START;
		SERIAL_ECHOPAIR("  G29 S3 X", (int)px + 1);
		SERIAL_ECHOPAIR(" Y", (int)py + 1);
		SERIAL_ECHOPGM(" Z");
		SERIAL_ECHO_F(LINEAR_UNIT(mbl.z_values[px][py]), 5);
		SERIAL_EOL();
            }
	}
    }
#elif ENABLED(AUTO_BED_LEVELING_UBL)
    if (!forReplay) {
	SERIAL_EOL();
	ubl.report_state();
	SERIAL_ECHOLNPAIR("\nActive Mesh Slot: ", ubl.storage_slot);
	SERIAL_ECHOPAIR("EEPROM can hold ", calc_num_meshes());
	SERIAL_ECHOLNPGM(" meshes.\n");
    }
#elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
    // ubl.report_current_mesh(PORTVAR_SOLO);
    // this is too verbose for large mesh's. A better (more terse)
    // solution needs to be found.
    if (leveling_is_valid()) {
	for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
		CONFIG_ECHO_START;
		SERIAL_ECHOPAIR("  G29 W I", (int)px);
		SERIAL_ECHOPAIR(" J", (int)py);
		SERIAL_ECHOPGM(" Z");
		SERIAL_ECHO_F(LINEAR_UNIT(z_values[px][py]), 5);
		SERIAL_EOL();
            }
	}
    }
#endif
#endif // HAS_LEVELING

#if ENABLED(DELTA)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Endstop adjustment:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X", LINEAR_UNIT(delta_endstop_adj[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(delta_endstop_adj[Y_AXIS]));
    SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(delta_endstop_adj[Z_AXIS]));
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Delta settings: L<diagonal_rod> R<radius> H<height> S<segments_per_s> B<calibration radius> XYZ<tower angle corrections>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M665 L", LINEAR_UNIT(delta_diagonal_rod));
    SERIAL_ECHOPAIR(" R", LINEAR_UNIT(delta_radius));
    SERIAL_ECHOPAIR(" H", LINEAR_UNIT(delta_height));
    SERIAL_ECHOPAIR(" S", delta_segments_per_second);
    SERIAL_ECHOPAIR(" B", LINEAR_UNIT(delta_calibration_radius));
    SERIAL_ECHOPAIR(" X", LINEAR_UNIT(delta_tower_angle_trim[A_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(delta_tower_angle_trim[B_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(delta_tower_angle_trim[C_AXIS]));
    SERIAL_EOL();
#elif ENABLED(HANGPRINTER)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Hangprinter settings: W<Ay> E<Az> R<Bx> T<By> Y<Bz> U<Cx> I<Cy> O<Cz> P<Dz> S<segments_per_s>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M665 W", anchor_A_y);
    SERIAL_ECHOPAIR(" E", anchor_A_z);
    SERIAL_ECHOPAIR(" R", anchor_B_x);
    SERIAL_ECHOPAIR(" T", anchor_B_y);
    SERIAL_ECHOPAIR(" Y", anchor_B_z);
    SERIAL_ECHOPAIR(" U", anchor_C_x);
    SERIAL_ECHOPAIR(" I", anchor_C_y);
    SERIAL_ECHOPAIR(" O", anchor_C_z);
    SERIAL_ECHOPAIR(" P", anchor_D_z);
    SERIAL_ECHOPAIR(" S", delta_segments_per_second);
    SERIAL_EOL();
#elif ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_DUAL_ENDSTOPS)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Endstop adjustment:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPGM("  M666");
#if ENABLED(X_DUAL_ENDSTOPS)
    SERIAL_ECHOPAIR(" X", LINEAR_UNIT(endstops.x_endstop_adj));
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(endstops.y_endstop_adj));
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(endstops.z_endstop_adj));
#endif
    SERIAL_EOL();
#endif // [XYZ]_DUAL_ENDSTOPS

#if ENABLED(ULTIPANEL)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Material heatup parameters:");
    }
    for (uint8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        CONFIG_ECHO_START;
        SERIAL_ECHOPAIR("  M145 S", (int)i);
        SERIAL_ECHOPAIR(" H", TEMP_UNIT(lcd_preheat_hotend_temp[i]));
        SERIAL_ECHOPAIR(" B", TEMP_UNIT(lcd_preheat_bed_temp[i]));
        SERIAL_ECHOLNPAIR(" F", lcd_preheat_fan_speed[i]);
    }
#endif // ULTIPANEL

#if HAS_PID_HEATING
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("PID settings:");
    }
#if ENABLED(PIDTEMP)
#if HOTENDS > 1
    if (forReplay) {
	HOTEND_LOOP() {
	    CONFIG_ECHO_START;
	    SERIAL_ECHOPAIR("  M301 E", e);
	    SERIAL_ECHOPAIR(" P", PID_PARAM(Kp, e));
	    SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, e)));
	    SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, e)));
#if ENABLED(PID_EXTRUSION_SCALING)
	    SERIAL_ECHOPAIR(" C", PID_PARAM(Kc, e));
	    if (e == 0) SERIAL_ECHOPAIR(" L", thermalManager.lpq_len);
#endif
	    SERIAL_EOL();
	}
    }
    else
#endif
	{
	// !forReplay || HOTENDS == 1
	CONFIG_ECHO_START;
	// for compatibility with hosts, only echo values for E0
	SERIAL_ECHOPAIR("  M301 P", PID_PARAM(Kp, 0));
	SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, 0)));
	SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, 0)));
#if ENABLED(PID_EXTRUSION_SCALING)
	SERIAL_ECHOPAIR(" C", PID_PARAM(Kc, 0));
	SERIAL_ECHOPAIR(" L", thermalManager.lpq_len);
#endif
	SERIAL_EOL();
    }
#endif // PIDTEMP
#if ENABLED(PIDTEMPBED)
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M304 P", thermalManager.bedKp);
    SERIAL_ECHOPAIR(" I", unscalePID_i(thermalManager.bedKi));
    SERIAL_ECHOPAIR(" D", unscalePID_d(thermalManager.bedKd));
    SERIAL_EOL();
#endif
#endif // PIDTEMP || PIDTEMPBED

#if HAS_LCD_CONTRAST
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("LCD Contrast:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOLNPAIR("  M250 C", lcd_contrast);
#endif

#if ENABLED(FWRETRACT)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Retract: S<length> F<units/m> Z<lift>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M207 S", LINEAR_UNIT(fwretract.retract_length));
    SERIAL_ECHOPAIR(" W", LINEAR_UNIT(fwretract.swap_retract_length));
    SERIAL_ECHOPAIR(" F",
      MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_feedrate_mm_s)));
    SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(fwretract.retract_zlift));
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Recover: S<length> F<units/m>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M208 S",
      LINEAR_UNIT(fwretract.retract_recover_length));
    SERIAL_ECHOPAIR(" W",
      LINEAR_UNIT(fwretract.swap_retract_recover_length));
    SERIAL_ECHOLNPAIR(" F",
      MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_recover_feedrate_mm_s)));
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM(
"Auto-Retract: S=0 to disable, 1 to interpret E-only moves as retract/recover"
			 );
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOLNPAIR("  M209 S", fwretract.autoretract_enabled ? 1 : 0);
#endif // FWRETRACT

    // Probe Offset
#if HAS_BED_PROBE
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOPGM("Z-Probe Offset");
        say_units(true);
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOLNPAIR("  M851 Z", LINEAR_UNIT(zprobe_zoffset));
#endif

    // Bed Skew Correction
#if ENABLED(SKEW_CORRECTION_GCODE)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Skew Factor: ");
    }
    CONFIG_ECHO_START;
#if ENABLED(SKEW_CORRECTION_FOR_Z)
    SERIAL_ECHOPGM("  M852 I");
    SERIAL_ECHO_F(LINEAR_UNIT(planner.xy_skew_factor), 6);
    SERIAL_ECHOPGM(" J");
    SERIAL_ECHO_F(LINEAR_UNIT(planner.xz_skew_factor), 6);
    SERIAL_ECHOPGM(" K");
    SERIAL_ECHO_F(LINEAR_UNIT(planner.yz_skew_factor), 6);
    SERIAL_EOL();
#else
    SERIAL_ECHOPGM("  M852 S");
    SERIAL_ECHO_F(LINEAR_UNIT(planner.xy_skew_factor), 6);
    SERIAL_EOL();
#endif
#endif

#if HAS_TRINAMIC
    // TMC2130 / TMC2208 stepper driver current
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Stepper driver current:");
    }
    CONFIG_ECHO_START;
#if AXIS_IS_TMC(X) || AXIS_IS_TMC(Y) || AXIS_IS_TMC(Z)
    say_M906();
#endif
#if AXIS_IS_TMC(X)
    SERIAL_ECHOPAIR(" X", stepperX.getCurrent());
#endif
#if AXIS_IS_TMC(Y)
    SERIAL_ECHOPAIR(" Y", stepperY.getCurrent());
#endif
#if AXIS_IS_TMC(Z)
    SERIAL_ECHOPAIR(" Z", stepperZ.getCurrent());
#endif
#if AXIS_IS_TMC(X) || AXIS_IS_TMC(Y) || AXIS_IS_TMC(Z)
    SERIAL_EOL();
#endif
#if AXIS_IS_TMC(X2) || AXIS_IS_TMC(Y2) || AXIS_IS_TMC(Z2)
    say_M906();
    SERIAL_ECHOPGM(" I1");
#endif
#if AXIS_IS_TMC(X2)
    SERIAL_ECHOPAIR(" X", stepperX2.getCurrent());
#endif
#if AXIS_IS_TMC(Y2)
    SERIAL_ECHOPAIR(" Y", stepperY2.getCurrent());
#endif
#if AXIS_IS_TMC(Z2)
    SERIAL_ECHOPAIR(" Z", stepperZ2.getCurrent());
#endif
#if AXIS_IS_TMC(X2) || AXIS_IS_TMC(Y2) || AXIS_IS_TMC(Z2)
    SERIAL_EOL();
#endif
#if AXIS_IS_TMC(E0)
    say_M906();
    SERIAL_ECHOLNPAIR(" T0 E", stepperE0.getCurrent());
#endif
#if E_STEPPERS > 1 && AXIS_IS_TMC(E1)
    say_M906();
    SERIAL_ECHOLNPAIR(" T1 E", stepperE1.getCurrent());
#endif
#if E_STEPPERS > 2 && AXIS_IS_TMC(E2)
    say_M906();
    SERIAL_ECHOLNPAIR(" T2 E", stepperE2.getCurrent());
#endif
#if E_STEPPERS > 3 && AXIS_IS_TMC(E3)
    say_M906();
    SERIAL_ECHOLNPAIR(" T3 E", stepperE3.getCurrent());
#endif
#if E_STEPPERS > 4 && AXIS_IS_TMC(E4)
    say_M906();
    SERIAL_ECHOLNPAIR(" T4 E", stepperE4.getCurrent());
#endif
    SERIAL_EOL();
    // TMC2130 / TMC2208 / TRAMS Hybrid Threshold
#if ENABLED(HYBRID_THRESHOLD)
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM("Hybrid Threshold:");
    }
    CONFIG_ECHO_START;
#if AXIS_IS_TMC(X) || AXIS_IS_TMC(Y) || AXIS_IS_TMC(Z)
    say_M913();
#endif
#if AXIS_IS_TMC(X)
    SERIAL_ECHOPAIR(" X", TMC_GET_PWMTHRS(X, X));
#endif
#if AXIS_IS_TMC(Y)
    SERIAL_ECHOPAIR(" Y", TMC_GET_PWMTHRS(Y, Y));
#endif
#if AXIS_IS_TMC(Z)
    SERIAL_ECHOPAIR(" Z", TMC_GET_PWMTHRS(Z, Z));
#endif
#if AXIS_IS_TMC(X) || AXIS_IS_TMC(Y) || AXIS_IS_TMC(Z)
    SERIAL_EOL();
#endif
#if AXIS_IS_TMC(X2) || AXIS_IS_TMC(Y2) || AXIS_IS_TMC(Z2)
    say_M913();
    SERIAL_ECHOPGM(" I1");
#endif
#if AXIS_IS_TMC(X2)
    SERIAL_ECHOPAIR(" X", TMC_GET_PWMTHRS(X, X2));
#endif
#if AXIS_IS_TMC(Y2)
    SERIAL_ECHOPAIR(" Y", TMC_GET_PWMTHRS(Y, Y2));
#endif
#if AXIS_IS_TMC(Z2)
    SERIAL_ECHOPAIR(" Z", TMC_GET_PWMTHRS(Z, Z2));
#endif
#if AXIS_IS_TMC(X2) || AXIS_IS_TMC(Y2) || AXIS_IS_TMC(Z2)
    SERIAL_EOL();
#endif
#if AXIS_IS_TMC(E0)
    say_M913();
    SERIAL_ECHOLNPAIR(" T0 E", TMC_GET_PWMTHRS(E, E0));
#endif
#if E_STEPPERS > 1 && AXIS_IS_TMC(E1)
    say_M913();
    SERIAL_ECHOLNPAIR(" T1 E", TMC_GET_PWMTHRS(E, E1));
#endif
#if E_STEPPERS > 2 && AXIS_IS_TMC(E2)
    say_M913();
    SERIAL_ECHOLNPAIR(" T2 E", TMC_GET_PWMTHRS(E, E2));
#endif
#if E_STEPPERS > 3 && AXIS_IS_TMC(E3)
    say_M913();
    SERIAL_ECHOLNPAIR(" T3 E", TMC_GET_PWMTHRS(E, E3));
#endif
#if E_STEPPERS > 4 && AXIS_IS_TMC(E4)
    say_M913();
    SERIAL_ECHOLNPAIR(" T4 E", TMC_GET_PWMTHRS(E, E4));
#endif
    SERIAL_EOL();
#endif // HYBRID_THRESHOLD
    // TMC2130 Sensorless homing thresholds
#if ENABLED(SENSORLESS_HOMING)
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM("Sensorless homing threshold:");
    }
    CONFIG_ECHO_START;
#if X_SENSORLESS || Y_SENSORLESS || Z_SENSORLESS
    say_M914();
#if X_SENSORLESS
    SERIAL_ECHOPAIR(" X", stepperX.sgt());
#endif
#if Y_SENSORLESS
    SERIAL_ECHOPAIR(" Y", stepperY.sgt());
#endif
#if Z_SENSORLESS
    SERIAL_ECHOPAIR(" Z", stepperZ.sgt());
#endif
    SERIAL_EOL();
#endif
	
#define X2_SENSORLESS (defined(X_HOMING_SENSITIVITY) &&AXIS_HAS_STALLGUARD(X2))
#define Y2_SENSORLESS (defined(Y_HOMING_SENSITIVITY) &&AXIS_HAS_STALLGUARD(Y2))
#define Z2_SENSORLESS (defined(Z_HOMING_SENSITIVITY) &&AXIS_HAS_STALLGUARD(Z2))
#if X2_SENSORLESS || Y2_SENSORLESS || Z2_SENSORLESS
    say_M914();
    SERIAL_ECHOPGM(" I1");
#if X2_SENSORLESS
    SERIAL_ECHOPAIR(" X", stepperX2.sgt());
#endif
#if Y2_SENSORLESS
    SERIAL_ECHOPAIR(" Y", stepperY2.sgt());
#endif
#if Z2_SENSORLESS
    SERIAL_ECHOPAIR(" Z", stepperZ2.sgt());
#endif
    SERIAL_EOL();
#endif
#endif // SENSORLESS_HOMING
#endif // HAS_TRINAMIC

	// Linear Advance
#if ENABLED(LIN_ADVANCE)
    if (!forReplay) {
	CONFIG_ECHO_START;
	SERIAL_ECHOLNPGM("Linear Advance:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOLNPAIR("  M900 K", planner.extruder_advance_K);
#endif
#if HAS_MOTOR_CURRENT_PWM
    CONFIG_ECHO_START;
    if (!forReplay) {
        SERIAL_ECHOLNPGM("Stepper motor currents:");
        CONFIG_ECHO_START;
    }
    SERIAL_ECHOPAIR("  M907 X", stepper.motor_current_setting[0]);
    SERIAL_ECHOPAIR(" Z", stepper.motor_current_setting[1]);
    SERIAL_ECHOPAIR(" E", stepper.motor_current_setting[2]);
    SERIAL_EOL();
#endif
    // Advanced Pause filament load & unload lengths
#if ENABLED(ADVANCED_PAUSE_FEATURE)
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Filament load/unload lengths:");
    }
    CONFIG_ECHO_START;
#if EXTRUDERS == 1
    say_M603();
    SERIAL_ECHOPAIR("L", LINEAR_UNIT(filament_change_load_length[0]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[0]));
#else
    say_M603();
    SERIAL_ECHOPAIR("T0 L", LINEAR_UNIT(filament_change_load_length[0]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[0]));
    CONFIG_ECHO_START;
    say_M603();
    SERIAL_ECHOPAIR("T1 L", LINEAR_UNIT(filament_change_load_length[1]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[1]));
#if EXTRUDERS > 2
    CONFIG_ECHO_START;
    say_M603();
    SERIAL_ECHOPAIR("T2 L", LINEAR_UNIT(filament_change_load_length[2]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[2]));
#if EXTRUDERS > 3
    CONFIG_ECHO_START;
    say_M603();
    SERIAL_ECHOPAIR("T3 L", LINEAR_UNIT(filament_change_load_length[3]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[3]));
#if EXTRUDERS > 4
    CONFIG_ECHO_START;
    say_M603();
    SERIAL_ECHOPAIR("T4 L", LINEAR_UNIT(filament_change_load_length[4]));
    SERIAL_ECHOLNPAIR(" U", LINEAR_UNIT(filament_change_unload_length[4]));
#endif // EXTRUDERS > 4
#endif // EXTRUDERS > 3
#endif // EXTRUDERS > 2
#endif // EXTRUDERS == 1
#endif // ADVANCED_PAUSE_FEATURE

#if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    // SD card sort options
    char s[32];
    if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("SD Card Sort Options:");
    }
    CONFIG_ECHO_START;
    sprintf(s, "  M34 S%d F%d", card.getSortAlpha(), card.getSortFolders());
    SERIAL_ECHOLN(s);
#endif
}
#endif // !DISABLE_M503
