/**
 * @file proto_parser.h
 * @brief Protocol parser for all incoming UDP messages
 *
 * Parses all message formats received from the server/other cars,
 * and exposes parsed results via accessor functions for downstream use
 * (e.g. sending motion commands to STM32 over UART).
 *
 * Supported message formats:
 *
 *  1. Topology enable/disable  [T,E,0|1]
 *  2. Topology matrix          [T,M,v00,v01,...,v33]   (16 values, row-major)
 *  3. Move command             [C,CARn,x,y,yaw]
 *  4. Formation stop           [F,T]
 *  5. Formation start          [F,S,CARn,type]
 *  6. Formation leader         [F,L,CARn]
 *  7. Formation follower       [F,F,CARn,off_x,off_y,off_yaw]
 *  8. Formation offset update  [F,U,CARn,off_x,off_y,off_yaw]
 *  9. Broadcast car state      [N C1 x y yaw vx vy vz C2 ...]
 * 10. Trajectory command       [T,CARn,vx,vy,target_yaw]
 * 11. Legacy topology matrix   TOPOLOGY:r0c0,r0c1,...;r1c0,...
 * 12. Legacy topology toggle   TOPOLOGY_TOGGLE:True|False
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Constants ========== */

#define PROTO_MAX_CARS      4    /**< Max number of cars in topology */
#define PROTO_CAR_ID_LEN    10   /**< Max length of a car ID string  */

/* ========== Data Types ========== */

/**
 * @brief Topology configuration (mirrors STM32 communication_topology)
 */
typedef struct {
    uint8_t matrix[PROTO_MAX_CARS][PROTO_MAX_CARS]; /**< A[i][j]=1: car-i can send to car-j */
    bool    enabled;                                 /**< topology filter active?             */
} proto_topology_t;

/**
 * @brief Move-to-position command for one car  ([C,...])
 */
typedef struct {
    char  target_car[PROTO_CAR_ID_LEN]; /**< e.g. "CAR1"  */
    float x;                            /**< target X (m)  */
    float y;                            /**< target Y (m)  */
    float yaw;                          /**< target yaw (deg) */
    bool  valid;                        /**< true when freshly parsed */
} proto_cmd_move_t;

/**
 * @brief Formation command types
 */
typedef enum {
    FORMATION_CMD_NONE     = 0,
    FORMATION_CMD_STOP,         /**< [F,T]             stop formation  */
    FORMATION_CMD_START,        /**< [F,S,leader,type] start formation */
    FORMATION_CMD_SET_LEADER,   /**< [F,L,CARn]        become leader   */
    FORMATION_CMD_SET_FOLLOWER, /**< [F,F,leader,ox,oy,oyaw] follower  */
    FORMATION_CMD_UPDATE_OFFSET /**< [F,U,leader,ox,oy,oyaw] update    */
} proto_formation_cmd_type_t;

/**
 * @brief Formation command payload
 */
typedef struct {
    proto_formation_cmd_type_t type;
    char  leader_id[PROTO_CAR_ID_LEN];
    float offset_x;
    float offset_y;
    float offset_yaw;
    bool  valid;
} proto_cmd_formation_t;

/**
 * @brief State of one remote car (from broadcast)
 */
typedef struct {
    char    car_id[PROTO_CAR_ID_LEN];
    float   x;
    float   y;
    float   yaw;    /**< heading angle (deg) */
    float   vx;
    float   vy;
    float   vz;
    uint32_t last_update_ms;  /**< timestamp from esp_timer / xTaskGetTickCount */
    bool    valid;
} proto_car_state_t;


/* ========== Initialisation ========== */

/**
 * @brief Initialise the parser module.
 *        Call once before proto_parse_message().
 *
 * @param own_car_id  This car's ID string (e.g. "CAR1").
 *                    Used to skip own state in broadcast packets.
 */
void proto_parser_init(const char *own_car_id);

/* ========== Main Parse Entry ========== */

/**
 * @brief Parse one complete UDP payload string.
 *
 * Identifies the message type and updates the corresponding internal
 * state.  Call from the UDP RX callback.
 *
 * @param data  Null-terminated payload string.
 * @param len   Byte length of data (excluding null terminator).
 */
void proto_parse_message(const char *data, size_t len);

/* ========== Topology Accessors ========== */

/**
 * @brief Get current topology configuration (read-only snapshot).
 * @return Pointer to internal topology struct (valid until next parse call).
 */
const proto_topology_t *proto_get_topology(void);

/**
 * @brief Check whether this car should accept data from a given sender.
 *
 * Implements the same rule as STM32 Should_Process_Car_Info():
 *   accept = topology.matrix[sender_index][own_index] == 1
 *   (or topology disabled → always accept)
 *
 * @param sender_car_id  e.g. "CAR2"
 * @return true if the data should be processed.
 */
bool proto_should_accept_from(const char *sender_car_id);

/* ========== Move Command Accessors ========== */

/**
 * @brief Consume the latest move command addressed to own_car_id.
 *
 * Returns a copy of the command and clears the "valid" flag so that
 * the same command is not consumed twice.
 *
 * @param[out] out  Filled with the command data.
 * @return true if a fresh command was available.
 */
bool proto_consume_move_cmd(proto_cmd_move_t *out);

/* ========== Formation Command Accessors ========== */

/**
 * @brief Consume the latest formation command.
 *
 * @param[out] out  Filled with the formation command data.
 * @return true if a fresh command was available.
 */
bool proto_consume_formation_cmd(proto_cmd_formation_t *out);

/* ========== Remote Car State Accessors ========== */

/**
 * @brief Get the state of a specific remote car.
 *
 * @param car_id  e.g. "CAR2"
 * @return Pointer to internal state struct, or NULL if not found.
 *         Valid only until next proto_parse_message() call.
 */
const proto_car_state_t *proto_get_car_state(const char *car_id);

/**
 * @brief Get the full array of tracked remote cars.
 *
 * @param[out] count  Filled with the number of valid entries.
 * @return Pointer to the internal array (PROTO_MAX_CARS entries).
 */
const proto_car_state_t *proto_get_all_car_states(int *count);

/**
 * @brief Remove entries that have not been updated within timeout_ms.
 *
 * Call periodically (e.g. every second) to clean stale entries.
 *
 * @param timeout_ms  Stale threshold in milliseconds (e.g. 3000).
 */
void proto_cleanup_stale_cars(uint32_t timeout_ms);

/* ========== Utility ========== */

/**
 * @brief Convert a car ID string to its 0-based index.
 *
 * "CAR1"->0, "CAR2"->1, "CAR3"->2, "CAR4"->3, unknown->-1
 */
int proto_car_id_to_index(const char *car_id);

#ifdef __cplusplus
}
#endif
