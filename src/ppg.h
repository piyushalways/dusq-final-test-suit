#ifndef PPG_H
#define PPG_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

/* 1. Define the states for PPG manager */
typedef enum {
    PPG_STATE_OFF,
    PPG_STATE_MEASURING,
} ppg_state_t;

/* 2. Define the commands for the message queue */
typedef enum {
    PPG_CMD_BLE_START,
    PPG_CMD_BLE_STOP,
    PPG_CMD_INTERNAL_FIFO_READY,
} ppg_cmd_t;

/* 3. Define the message queue */
// The message queue will hold one 'ppg_cmd_t' message at a time.
extern struct k_msgq ppg_cmd_q;

/* 3a. Dedicated flash workqueue for flash I/O operations
 * Use this instead of system workqueue for flash operations to prevent
 * blocking BLE and other time-sensitive tasks during 50-100ms flash writes.
 */
extern struct k_work_q flash_workqueue;

/* 4. BLE functions - separate characteristics in same service */
extern void ble_send_ppg_data(const uint8_t *data, uint16_t length);        // Raw PPG data characteristic
extern void ble_send_hrv_features_data(const uint8_t *data, uint16_t length); // HRV features characteristic

/* 5. Flash read pause/resume functions */
extern void ppg_pause_for_flash_read(void);   // Pause PPG processing during flash read
extern void ppg_resume_after_flash_read(void); // Resume PPG processing after flash read

/* 6. Flash wear leveling statistics (for diagnostics) */
extern void ppg_flash_get_wear_stats(uint32_t *out_min_writes, uint32_t *out_max_writes,
                                      uint32_t *out_total_sectors, uint32_t *out_valid_sectors);

/* 7. Two-pointer system: Get write head position for BLE status */
extern void ppg_flash_get_write_head(uint32_t *out_write_head_sector,
                                      uint32_t *out_write_cycle_count,
                                      uint32_t *out_total_samples_written);

/* 8. Data loss tracking for 40Hz sample rate validation (end-to-end) */
/**
 * @brief Get end-to-end data loss statistics
 *
 * Monitors complete PPG data path: AS7058 FIFO -> Callback -> Buffer -> Flash
 * Use this for a quick summary of data integrity.
 *
 * @param out_received     Total samples received from FIFO (NULL to skip)
 * @param out_expected     Expected samples based on elapsed time (NULL to skip)
 * @param out_lost         Total samples lost across all stages (NULL to skip)
 * @param out_gap_count    Number of FIFO timing gap events (NULL to skip)
 * @param out_loss_percent End-to-end loss percentage 0-100 (NULL to skip)
 */
extern void ppg_get_data_loss_stats(uint32_t *out_received, uint32_t *out_expected,
                                     uint32_t *out_lost, uint32_t *out_gap_count,
                                     float *out_loss_percent);

/**
 * @brief Get detailed data loss statistics for each pipeline stage
 *
 * Provides granular visibility into where data loss occurs:
 *   Stage 1-2: FIFO -> Callback (timing gaps)
 *   Stage 2-3: Callback -> Buffer (drops when both buffers busy)
 *   Stage 3-4: Buffer -> Flash (write failures)
 *
 * @param out_fifo_received   Samples extracted from AS7058 FIFO (NULL to skip)
 * @param out_fifo_expected   Expected samples based on time at 40Hz (NULL to skip)
 * @param out_fifo_gaps       Number of timing gap events (NULL to skip)
 * @param out_buffer_stored   Samples successfully stored in ping-pong buffer (NULL to skip)
 * @param out_buffer_dropped  Samples dropped (both buffers busy) (NULL to skip)
 * @param out_flash_written   Samples successfully written to flash (NULL to skip)
 * @param out_flash_failed    Samples in failed flash write operations (NULL to skip)
 */
extern void ppg_get_detailed_data_loss_stats(
    uint32_t *out_fifo_received, uint32_t *out_fifo_expected, uint32_t *out_fifo_gaps,
    uint32_t *out_buffer_stored, uint32_t *out_buffer_dropped,
    uint32_t *out_flash_written, uint32_t *out_flash_failed);

#endif // PPG_H
