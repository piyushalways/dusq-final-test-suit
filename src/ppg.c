#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "as7058_chiplib.h"
#include "as7058_extract.h"
#include "error_codes.h"
#include "ppg.h"
#include "bluetooth_manager.h"
#include "ppg_workspace_f.h"
#include "core_modules_f.h"
#include "utility_f.h"
#include "bma580_features.h"
#include "bma5.h"
#include "stim_logic_f.h"

K_MSGQ_DEFINE(ppg_cmd_q, sizeof(ppg_cmd_t), 10, 4);

/******************************************************************************
 *                    BMA580 ACCELEROMETER CONFIGURATION                      *
 ******************************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! BMA580 I2C address */
#define BMA580_I2C_ADDR  0x18

/* I2C device pointer for BMA580 */
static const struct device *bma_i2c_dev = NULL;

/* BMA580 device structure */
static struct bma5_dev bma_dev;

/******************************************************************************
 *                       FLASH STORAGE CONFIGURATION                          *
 ******************************************************************************/

/* PPG data flash storage configuration */
#define PAGE_SIZE               256
#define FLASH_SECTOR_SIZE       4096          /* 4KB sector size for MX25R64 */

/* Define the total flash region for storing all our chunks */
#define DATA_REGION_START_OFFSET 0x000000      /* Start of our data log */
#define DATA_REGION_TOTAL_SIZE  8388608u      /* 8,388,608 bytes (8 MB) total for logging */

/* How much to print */
#define DUMP_HEAD_COUNT         20            /* first N numbers to print */
#define DUMP_TAIL_COUNT         10            /* last  N numbers to print */
#define WRITE_PROGRESS_BYTES    (16u * 1024u) /* print every 16 KiB written */
#define ERASE_CHUNK_BYTES       (32u * 1024u) /* erase in 32 KiB chunks */

/******************************************************************************
 *                    FLASH SECTOR ROTATION (WEAR LEVELING)                   *
 ******************************************************************************/

/*
 * Sector Header Structure - stored at the beginning of each sector
 * This enables wear leveling by tracking write counts per sector
 */
#define SECTOR_HEADER_MAGIC     0x50504731    /* "PPG1" */
#define SECTOR_HEADER_SIZE      32            /* Aligned to 32 bytes */

typedef struct __attribute__((packed)) {
    uint32_t magic;           /* Magic number to identify valid sector */
    uint32_t sequence_num;    /* Monotonically increasing sequence number */
    uint32_t write_count;     /* Number of times this sector has been written */
    uint32_t sample_count;    /* Number of valid samples in this sector */
    uint32_t timestamp;       /* Uptime in ms when written (for debugging) */
    uint32_t checksum;        /* Simple XOR checksum of first 5 fields */
    uint32_t reserved[2];     /* Reserved for future use */
} sector_header_t;

#define PPG_ELEM_SIZE           sizeof(uint32_t)

/* Calculate actual data capacity per sector (excluding header) */
#define SECTOR_DATA_CAPACITY    (FLASH_SECTOR_SIZE - SECTOR_HEADER_SIZE)
#define SAMPLES_PER_SECTOR      (SECTOR_DATA_CAPACITY / PPG_ELEM_SIZE)

/*
 * PPG RAM buffer size - matches sector data capacity for efficient writes
 * (4096 - 32) / 4 = 1016 samples per sector
 */
#define PPG_RAM_BUFFER_SIZE     SAMPLES_PER_SECTOR

/* Number of sectors in the data region */
#define NUM_SECTORS             (DATA_REGION_TOTAL_SIZE / FLASH_SECTOR_SIZE)

/* Wear leveling threshold - rotate when write count difference exceeds this */
#define WEAR_LEVEL_THRESHOLD    10

/* Devicetree and Sizing */
#if DT_HAS_COMPAT_STATUS_OKAY(nordic_qspi_nor)
#define FLASH_COMPAT nordic_qspi_nor
#elif DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor)
#define FLASH_COMPAT jedec_spi_nor
#elif DT_HAS_COMPAT_STATUS_OKAY(jedec_mspi_nor)
#define FLASH_COMPAT jedec_mspi_nor
#else
#warning "No compatible SPI/QSPI NOR flash found - flash storage will be disabled"
#define FLASH_STORAGE_AVAILABLE 0
#endif

#ifndef FLASH_STORAGE_AVAILABLE
#define FLASH_STORAGE_AVAILABLE 1
#endif

#define ROUND_UP(x,a)           ((((x) + ((a) - 1)) / (a)) * (a))

LOG_MODULE_REGISTER(ppg_manager, LOG_LEVEL_INF);

/******************************************************************************
 *                    BLE SERVICE - TWO CHARACTERISTICS                       *
 ******************************************************************************/
/*
 * Data is transmitted via a single BLE service with TWO characteristics:
 *
 * CHARACTERISTIC 1: Raw PPG Data (ble_send_ppg_data)
 *   Format: [SAMPLE1][SAMPLE2]...[SAMPLE_N]
 *   - Each SAMPLE: 4 bytes (int32_t, big-endian)
 *   - Up to 48 samples per packet
 *   - Total size: N × 4 bytes (max 192 bytes)
 *   - Update rate: ~40 Hz (every FIFO threshold)
 *
 * CHARACTERISTIC 2: HRV Features (ble_send_hrv_features_data)
 *   Format: [SQI][HR][SDNN][RMSSD][IBI][ACT]
 *   - SQI:   4 bytes (float, little-endian) - Signal Quality Index (0.0-1.0)
 *   - HR:    4 bytes (float, little-endian) - Heart Rate (BPM)
 *   - SDNN:  4 bytes (float, little-endian) - Std dev of NN intervals (ms)
 *   - RMSSD: 4 bytes (float, little-endian) - Root mean square successive diff (ms)
 *   - IBI:   4 bytes (float, little-endian) - Inter-beat interval mean (ms)
 *   - ACT:   1 byte  (uint8_t) - Activity level
 *   - Total size: 21 bytes
 *   - Update rate: Every 5 seconds
 */

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

/*! Metadata for storing state of FIFO data extraction. */
static volatile as7058_extract_metadata_t g_extract_metadata;

/*! Sample period of PPG1 and PPG2 signals. */
static volatile double g_ppg_sample_period_s;

/*! The total number of sub-samples received. */
static volatile uint32_t g_sub_sample_cnt[AS7058_SUB_SAMPLE_ID_NUM];

/******************************************************************************
 *                          HRM ALGORITHM GLOBALS                             *
 ******************************************************************************/

/* HRM workspace for processing */
static ppg_workspace_f_t ws;

/* Ring buffers for overlapping window processing */
static float ppg_ring[WIN_SAMP_PPG];
static float ax_ring[WIN_SAMP_ACC];
static float ay_ring[WIN_SAMP_ACC];
static float az_ring[WIN_SAMP_ACC];

/* Ring buffer write indices */
static int ppg_wr = 0;
static int acc_wr = 0;

/* Sample counters */
static int ppg_samples_collected = 0;
static int acc_samples_collected = 0;

/* Processing configuration */
#define STEP_SEC   5
#define STEP_PPG   (STEP_SEC * FS_PPG)
#define STEP_ACC   (STEP_SEC * FS_ACC)

/******************************************************************************
 *                    DATA LOSS TRACKING (40Hz VALIDATION)                    *
 ******************************************************************************
 *
 * COMPLETE DATA PATH MONITORING:
 *
 *   AS7058 FIFO  -->  Callback  -->  Ping-Pong Buffer  -->  External Flash
 *        |               |                  |                     |
 *   [STAGE 1]       [STAGE 2]          [STAGE 3]             [STAGE 4]
 *   FIFO burst    Samples recv'd     Buffer stored        Flash written
 *
 * Tracks:
 * - Stage 1-2: Sample arrival timing (expected 25ms intervals at 40Hz)
 * - Stage 2-3: Samples successfully buffered vs dropped (both buffers busy)
 * - Stage 3-4: Samples successfully written to flash vs write failures
 *
 * Expected behavior at 40Hz:
 * - 1 sample every 25ms, 40 samples per second
 * - FIFO callback delivers ~40 samples per burst (threshold=39)
 */

/* Expected sample rate in Hz */
#define PPG_EXPECTED_SAMPLE_RATE_HZ     40

/* Expected inter-sample period in milliseconds */
#define PPG_EXPECTED_SAMPLE_PERIOD_MS   (1000 / PPG_EXPECTED_SAMPLE_RATE_HZ)  /* 25ms */

/* Tolerance for gap detection: allow 50% longer than expected before flagging */
#define PPG_GAP_TOLERANCE_FACTOR        1.5f

/* Maximum acceptable gap before declaring data loss (ms) */
#define PPG_MAX_GAP_MS                  ((uint32_t)(PPG_EXPECTED_SAMPLE_PERIOD_MS * PPG_GAP_TOLERANCE_FACTOR))

/* Reporting interval for data loss statistics (ms) */
#define PPG_DATA_LOSS_REPORT_INTERVAL_MS  10000  /* Report every 10 seconds */

/* Data loss tracking structure - tracks complete data path */
typedef struct {
    /*=========================================================================
     * STAGE 1-2: FIFO to Callback (Sample Arrival)
     *=========================================================================*/
    /* Timing tracking */
    uint64_t first_sample_time_ms;      /* Timestamp of first sample received */
    uint64_t last_sample_time_ms;       /* Timestamp of most recent sample */
    uint64_t last_report_time_ms;       /* Last time stats were reported */

    /* Sample counting from FIFO */
    uint32_t fifo_samples_received;     /* Total samples extracted from FIFO */
    uint32_t fifo_samples_expected;     /* Expected samples based on elapsed time */
    uint32_t samples_since_last_report; /* Samples received since last report */

    /* Gap/loss detection at FIFO level */
    uint32_t fifo_gap_count;            /* Number of detected timing gaps */
    uint32_t fifo_max_gap_ms;           /* Largest gap detected (ms) */
    uint32_t fifo_samples_lost;         /* Estimated samples lost (timing-based) */

    /* FIFO burst tracking */
    uint32_t fifo_callback_count;       /* Number of FIFO callbacks received */
    uint32_t min_samples_per_burst;     /* Minimum samples in a single callback */
    uint32_t max_samples_per_burst;     /* Maximum samples in a single callback */

    /*=========================================================================
     * STAGE 2-3: Callback to Ping-Pong Buffer
     *=========================================================================*/
    uint32_t buffer_samples_stored;     /* Samples successfully added to buffer */
    uint32_t buffer_samples_dropped;    /* Samples dropped (both buffers busy) */
    uint32_t buffer_swap_count;         /* Number of buffer swaps */

    /*=========================================================================
     * STAGE 3-4: Ping-Pong Buffer to Flash
     *=========================================================================*/
    uint32_t flash_samples_written;     /* Samples successfully written to flash */
    uint32_t flash_write_count;         /* Number of successful flash writes */
    uint32_t flash_write_failures;      /* Number of failed flash writes */
    uint32_t flash_samples_failed;      /* Samples in failed write attempts */

    /*=========================================================================
     * Session State
     *=========================================================================*/
    bool tracking_active;               /* Is tracking enabled */
    bool first_sample_received;         /* Has first sample arrived */
} ppg_data_loss_tracker_t;

/* Global data loss tracker instance */
static ppg_data_loss_tracker_t g_data_loss_tracker = {0};

/******************************************************************************
 *                      FLASH STORAGE GLOBALS                                 *
 ******************************************************************************/

#if FLASH_STORAGE_AVAILABLE
/* Flash device pointer */
static const struct device *flash_dev = NULL;

/******************************************************************************
 *                    PING-PONG DOUBLE BUFFER SYSTEM                          *
 ******************************************************************************
 *
 * Two 4KB buffers (PING and PONG) enable continuous data acquisition:
 *
 *   Time -->
 *   ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
 *   │ PING: Filling   │  │ PING: Offload   │  │ PING: Filling   │
 *   │ PONG: Idle      │  │ PONG: Filling   │  │ PONG: Offload   │
 *   └─────────────────┘  └─────────────────┘  └─────────────────┘
 *
 * Benefits:
 * - Zero data loss during flash writes (flash write takes ~50-100ms)
 * - Continuous PPG sampling at 100Hz without gaps
 * - Each buffer holds 1016 samples (matches sector data capacity)
 *
 * Buffer State Machine:
 *   IDLE -> FILLING -> FULL -> OFFLOADING -> IDLE
 */

/* Buffer states */
typedef enum {
    BUF_STATE_IDLE = 0,       /* Buffer is empty and ready */
    BUF_STATE_FILLING,        /* Currently receiving PPG samples */
    BUF_STATE_FULL,           /* Buffer is full, waiting for offload */
    BUF_STATE_OFFLOADING      /* Buffer is being written to flash */
} ppg_buffer_state_t;

/* Ping-Pong buffer identifiers */
typedef enum {
    PPG_BUF_PING = 0,
    PPG_BUF_PONG = 1
} ppg_buffer_id_t;

/* Double buffer structure */
typedef struct {
    uint32_t data[PPG_RAM_BUFFER_SIZE];   /* Sample data (1016 samples = ~4KB) */
    size_t sample_count;                   /* Number of valid samples */
    ppg_buffer_state_t state;              /* Current buffer state */
    uint32_t sector_index;                 /* Target sector for this buffer */
    off_t write_offset;                    /* Target flash offset */
} ppg_pingpong_buffer_t;

/* The two buffers */
static ppg_pingpong_buffer_t ppg_buffers[2];

/* Which buffer is currently being filled */
static volatile ppg_buffer_id_t active_buffer = PPG_BUF_PING;

/******************************************************************************
 *                    DEDICATED FLASH WORKQUEUE                               *
 ******************************************************************************
 * Isolated workqueue for flash operations to prevent blocking system workqueue.
 * Flash writes can take 50-100ms which would starve BLE and other tasks.
 * Priority 5 = lower than BLE (1) but higher than idle.
 */
#define FLASH_WORKQUEUE_STACK_SIZE 2048
#define FLASH_WORKQUEUE_PRIORITY   5

K_THREAD_STACK_DEFINE(flash_workqueue_stack, FLASH_WORKQUEUE_STACK_SIZE);
struct k_work_q flash_workqueue;  /* Exported for bluetooth_manager.c */

/* Work item for background flash offload */
static struct k_work_delayable flash_offload_work;

/* Semaphore to signal offload completion */
K_SEM_DEFINE(offload_complete_sem, 0, 1);

/* Statistics for monitoring */
static uint32_t buffer_swap_count = 0;
static uint32_t offload_in_progress_drops = 0;  /* Samples dropped due to both buffers busy */

/* Legacy compatibility - pointer to active buffer's sample count */
#define ppg_flash_sample_count (ppg_buffers[active_buffer].sample_count)

/******************************************************************************
 *                    TWO-POINTER SYSTEM FOR FLASH MANAGEMENT                 *
 ******************************************************************************
 *
 * POINTER 1: WRITE HEAD (write_head_sector)
 * -----------------------------------------
 * - Points to the next sector where data will be written
 * - Advances SEQUENTIALLY through all sectors (0 -> 1 -> 2 -> ... -> N-1 -> 0)
 * - This circular advancement ensures PERFECTLY EVEN WEAR across all sectors
 * - Each full cycle through all sectors = exactly 1 write to each sector
 * - Persisted to flash settings so it survives reboots
 *
 * POINTER 2: BLE SYNC POINTER (in bluetooth_manager.c as transfer_progress)
 * -------------------------------------------------------------------------
 * - Tracks the last offset successfully transferred over BLE
 * - Allows resuming interrupted transfers
 * - Independent of write head - can read old data while new data is written
 *
 * WEAR LEVELING GUARANTEE:
 * - All sectors receive the same number of writes (within 1 write difference)
 * - write_cycle_count tracks how many full cycles have completed
 * - max_writes = write_cycle_count + 1 for sectors before write_head
 * - max_writes = write_cycle_count for sectors at or after write_head
 */

/* POINTER 1: Write Head - next sector to write (sequential, circular) */
static uint32_t write_head_sector = 0;

/* Track number of complete write cycles through all sectors */
static uint32_t write_cycle_count = 0;

/* Flash write offset derived from write_head_sector */
static off_t current_write_offset = 0;
static uint32_t current_sector_index = 0;

/* Track total samples written to flash across all chunks */
static uint32_t total_flash_samples_written = 0;

/* Global sequence number for ordering sectors (monotonically increasing) */
static uint32_t global_sequence_num = 0;

/*
 * Sector wear tracking - stores write count for each sector
 * With sequential writes, this should be very uniform:
 * - Sectors before write_head: write_cycle_count + 1
 * - Sectors at/after write_head: write_cycle_count
 */
static uint16_t sector_write_counts[NUM_SECTORS];

/* Track which sectors have valid data (bitmap would be more efficient but this is clearer) */
static uint8_t sector_valid[NUM_SECTORS];

/* Statistics for wear leveling */
static uint32_t min_write_count = 0;
static uint32_t max_write_count = 0;
static uint32_t total_sectors_used = 0;

/* Flag to indicate if we should save to flash */
static volatile bool ppg_flash_storage_enabled = false;
/* Note: ppg_flash_buffer_full removed - ping-pong handles buffer state automatically */

/* Flag to pause PPG processing during flash read operations */
static volatile bool ppg_paused_for_flash_read = false;

/* Flag to indicate if sector tracking has been initialized */
static bool sector_tracking_initialized = false;

/* Flag to indicate if write head needs to be persisted */
static bool write_head_dirty = false;
#endif

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                    DATA LOSS TRACKING FUNCTIONS                            *
 ******************************************************************************/

/*!
 * @brief Reset data loss tracker to initial state
 *
 * Call this when starting a new measurement session to reset all counters.
 */
static void data_loss_tracker_reset(void)
{
    memset(&g_data_loss_tracker, 0, sizeof(g_data_loss_tracker));
    g_data_loss_tracker.min_samples_per_burst = UINT32_MAX;
    g_data_loss_tracker.tracking_active = false;
    g_data_loss_tracker.first_sample_received = false;
    LOG_INF("DATA_LOSS: Tracker reset");
}

/*!
 * @brief Start data loss tracking
 *
 * Call this when PPG measurement begins.
 */
static void data_loss_tracker_start(void)
{
    data_loss_tracker_reset();
    g_data_loss_tracker.tracking_active = true;
    LOG_INF("DATA_LOSS: Tracking started (expecting %d Hz)", PPG_EXPECTED_SAMPLE_RATE_HZ);
    LOG_INF("DATA_LOSS: Monitoring: FIFO -> Buffer -> Flash");
}

/*!
 * @brief Stop data loss tracking and log final statistics
 */
static void data_loss_tracker_stop(void)
{
    if (!g_data_loss_tracker.tracking_active) {
        return;
    }

    g_data_loss_tracker.tracking_active = false;

    /* Calculate session duration */
    uint64_t duration_ms = g_data_loss_tracker.last_sample_time_ms -
                           g_data_loss_tracker.first_sample_time_ms;
    float duration_sec = duration_ms / 1000.0f;

    /* Calculate loss rates for each stage */
    float fifo_loss_rate = 0.0f;
    float buffer_loss_rate = 0.0f;
    float flash_loss_rate = 0.0f;
    float end_to_end_loss_rate = 0.0f;

    if (g_data_loss_tracker.fifo_samples_expected > 0) {
        int32_t fifo_diff = (int32_t)g_data_loss_tracker.fifo_samples_expected -
                           (int32_t)g_data_loss_tracker.fifo_samples_received;
        fifo_loss_rate = (fifo_diff > 0) ?
            ((float)fifo_diff / (float)g_data_loss_tracker.fifo_samples_expected * 100.0f) : 0.0f;
    }

    if (g_data_loss_tracker.fifo_samples_received > 0) {
        buffer_loss_rate = (float)g_data_loss_tracker.buffer_samples_dropped /
                          (float)g_data_loss_tracker.fifo_samples_received * 100.0f;
    }

    if (g_data_loss_tracker.buffer_samples_stored > 0) {
        flash_loss_rate = (float)g_data_loss_tracker.flash_samples_failed /
                         (float)g_data_loss_tracker.buffer_samples_stored * 100.0f;
    }

    /*
     * End-to-end: True data loss = dropped + failed writes
     * Samples still in buffer when stopped are "pending" not "lost"
     */
    uint32_t samples_pending_in_buffer = g_data_loss_tracker.buffer_samples_stored -
                                         g_data_loss_tracker.flash_samples_written -
                                         g_data_loss_tracker.flash_samples_failed;
    uint32_t total_confirmed_loss = g_data_loss_tracker.buffer_samples_dropped +
                                    g_data_loss_tracker.flash_samples_failed;

    if (g_data_loss_tracker.fifo_samples_received > 0) {
        end_to_end_loss_rate = ((float)total_confirmed_loss /
                               (float)g_data_loss_tracker.fifo_samples_received) * 100.0f;
    }

    LOG_INF("DATA_LOSS: ╔══════════════════════════════════════════════════════════╗");
    LOG_INF("DATA_LOSS: ║          FINAL SESSION STATISTICS                        ║");
    LOG_INF("DATA_LOSS: ╠══════════════════════════════════════════════════════════╣");
    LOG_INF("DATA_LOSS: ║ Duration: %.1f seconds                                   ", (double)duration_sec);
    LOG_INF("DATA_LOSS: ╠══════════════════════════════════════════════════════════╣");
    LOG_INF("DATA_LOSS: ║ STAGE 1-2: AS7058 FIFO -> Callback                       ║");
    LOG_INF("DATA_LOSS: ║   Expected (by time): %u samples", g_data_loss_tracker.fifo_samples_expected);
    LOG_INF("DATA_LOSS: ║   Received from FIFO: %u samples", g_data_loss_tracker.fifo_samples_received);
    LOG_INF("DATA_LOSS: ║   Timing gaps: %u (max: %u ms)", g_data_loss_tracker.fifo_gap_count, g_data_loss_tracker.fifo_max_gap_ms);
    LOG_INF("DATA_LOSS: ║   FIFO loss rate: %.2f%%", (double)fifo_loss_rate);
    LOG_INF("DATA_LOSS: ║   Callbacks: %u (burst: %u-%u samples)",
            g_data_loss_tracker.fifo_callback_count,
            (g_data_loss_tracker.min_samples_per_burst == UINT32_MAX) ? 0 : g_data_loss_tracker.min_samples_per_burst,
            g_data_loss_tracker.max_samples_per_burst);
    LOG_INF("DATA_LOSS: ╠══════════════════════════════════════════════════════════╣");
    LOG_INF("DATA_LOSS: ║ STAGE 2-3: Callback -> Ping-Pong Buffer                  ║");
    LOG_INF("DATA_LOSS: ║   Stored in buffer: %u samples", g_data_loss_tracker.buffer_samples_stored);
    LOG_INF("DATA_LOSS: ║   Dropped (busy): %u samples", g_data_loss_tracker.buffer_samples_dropped);
    LOG_INF("DATA_LOSS: ║   Buffer swaps: %u", g_data_loss_tracker.buffer_swap_count);
    LOG_INF("DATA_LOSS: ║   Buffer loss rate: %.2f%%", (double)buffer_loss_rate);
    LOG_INF("DATA_LOSS: ╠══════════════════════════════════════════════════════════╣");
    LOG_INF("DATA_LOSS: ║ STAGE 3-4: Ping-Pong Buffer -> External Flash            ║");
    LOG_INF("DATA_LOSS: ║   Written to flash: %u samples", g_data_loss_tracker.flash_samples_written);
    LOG_INF("DATA_LOSS: ║   Still pending in buffer: %u samples", samples_pending_in_buffer);
    LOG_INF("DATA_LOSS: ║   Write operations: %u success, %u failed",
            g_data_loss_tracker.flash_write_count, g_data_loss_tracker.flash_write_failures);
    LOG_INF("DATA_LOSS: ║   Samples in failed writes: %u", g_data_loss_tracker.flash_samples_failed);
    LOG_INF("DATA_LOSS: ║   Flash loss rate: %.2f%%", (double)flash_loss_rate);
    LOG_INF("DATA_LOSS: ╠══════════════════════════════════════════════════════════╣");
    LOG_INF("DATA_LOSS: ║ END-TO-END SUMMARY                                       ║");
    LOG_INF("DATA_LOSS: ║   FIFO received: %u samples", g_data_loss_tracker.fifo_samples_received);
    LOG_INF("DATA_LOSS: ║   Flash written: %u samples", g_data_loss_tracker.flash_samples_written);
    LOG_INF("DATA_LOSS: ║   Pending flush: %u samples (in buffer, not lost)", samples_pending_in_buffer);
    LOG_INF("DATA_LOSS: ║   Confirmed lost: %u samples (dropped + failed)", total_confirmed_loss);
    LOG_INF("DATA_LOSS: ║   TRUE LOSS RATE: %.2f%%", (double)end_to_end_loss_rate);
    LOG_INF("DATA_LOSS: ╚══════════════════════════════════════════════════════════╝");
}

/*!
 * @brief Process incoming samples from FIFO (Stage 1-2)
 *
 * @param sample_count Number of samples received in this FIFO burst
 *
 * This function should be called each time samples are received from the
 * AS7058 FIFO. It tracks timing, detects gaps, and estimates lost samples.
 */
static void data_loss_tracker_process_fifo_samples(uint16_t sample_count)
{
    if (!g_data_loss_tracker.tracking_active || sample_count == 0) {
        return;
    }

    uint64_t now_ms = k_uptime_get();

    /* First sample initialization */
    if (!g_data_loss_tracker.first_sample_received) {
        g_data_loss_tracker.first_sample_time_ms = now_ms;
        g_data_loss_tracker.last_sample_time_ms = now_ms;
        g_data_loss_tracker.last_report_time_ms = now_ms;
        g_data_loss_tracker.first_sample_received = true;
        g_data_loss_tracker.fifo_samples_received = sample_count;
        g_data_loss_tracker.samples_since_last_report = sample_count;
        g_data_loss_tracker.fifo_callback_count = 1;
        g_data_loss_tracker.min_samples_per_burst = sample_count;
        g_data_loss_tracker.max_samples_per_burst = sample_count;
        return;
    }

    /* Calculate time since last sample batch */
    uint32_t gap_ms = (uint32_t)(now_ms - g_data_loss_tracker.last_sample_time_ms);

    /* Update timing */
    g_data_loss_tracker.last_sample_time_ms = now_ms;
    g_data_loss_tracker.fifo_samples_received += sample_count;
    g_data_loss_tracker.samples_since_last_report += sample_count;
    g_data_loss_tracker.fifo_callback_count++;

    /* Update burst statistics */
    if (sample_count < g_data_loss_tracker.min_samples_per_burst) {
        g_data_loss_tracker.min_samples_per_burst = sample_count;
    }
    if (sample_count > g_data_loss_tracker.max_samples_per_burst) {
        g_data_loss_tracker.max_samples_per_burst = sample_count;
    }

    /* Calculate expected samples based on elapsed time */
    uint64_t elapsed_ms = now_ms - g_data_loss_tracker.first_sample_time_ms;
    g_data_loss_tracker.fifo_samples_expected =
        (uint32_t)((elapsed_ms * PPG_EXPECTED_SAMPLE_RATE_HZ) / 1000);

    /* Calculate how many samples we should have received in this gap */
    uint32_t expected_in_gap = (gap_ms * PPG_EXPECTED_SAMPLE_RATE_HZ) / 1000;

    /* Detect gaps: if actual samples < expected for this time period */
    if (expected_in_gap > sample_count) {
        uint32_t lost_in_gap = expected_in_gap - sample_count;

        /* Only count as gap if significant (more than 1 sample lost) */
        if (lost_in_gap > 1) {
            g_data_loss_tracker.fifo_gap_count++;
            g_data_loss_tracker.fifo_samples_lost += lost_in_gap;

            if (gap_ms > g_data_loss_tracker.fifo_max_gap_ms) {
                g_data_loss_tracker.fifo_max_gap_ms = gap_ms;
            }

            LOG_WRN("DATA_LOSS: FIFO gap! Duration: %u ms, Expected: %u, Got: %u, Lost: %u",
                    gap_ms, expected_in_gap, sample_count, lost_in_gap);
        }
    }

    /* Periodic reporting */
    if ((now_ms - g_data_loss_tracker.last_report_time_ms) >= PPG_DATA_LOSS_REPORT_INTERVAL_MS) {
        /* Calculate loss rates */
        uint32_t expected_since_report =
            (PPG_DATA_LOSS_REPORT_INTERVAL_MS * PPG_EXPECTED_SAMPLE_RATE_HZ) / 1000;
        int32_t fifo_diff = (int32_t)expected_since_report -
                           (int32_t)g_data_loss_tracker.samples_since_last_report;
        float fifo_loss = (fifo_diff > 0 && expected_since_report > 0) ?
            ((float)fifo_diff / (float)expected_since_report * 100.0f) : 0.0f;

        /*
         * End-to-end calculation: account for samples still in ping-pong buffer
         * True loss = buffer_dropped + flash_failed (samples that will never make it)
         * Samples in active buffer are NOT lost, just pending flush
         */
        uint32_t samples_in_buffer = g_data_loss_tracker.buffer_samples_stored -
                                     g_data_loss_tracker.flash_samples_written -
                                     g_data_loss_tracker.flash_samples_failed;
        uint32_t confirmed_written_or_buffered = g_data_loss_tracker.flash_samples_written + samples_in_buffer;
        uint32_t actual_loss = g_data_loss_tracker.buffer_samples_dropped +
                               g_data_loss_tracker.flash_samples_failed;
        float e2e_loss = 0.0f;
        if (g_data_loss_tracker.buffer_samples_stored > 0) {
            e2e_loss = ((float)actual_loss / (float)g_data_loss_tracker.fifo_samples_received) * 100.0f;
        }

        LOG_INF("DATA_LOSS: [%llu s] FIFO: %u | Buf: %u stored, %u drop | Flash: %u written, %u pending | Loss: %u (%.2f%%)",
                (unsigned long long)(elapsed_ms / 1000),
                g_data_loss_tracker.fifo_samples_received,
                g_data_loss_tracker.buffer_samples_stored,
                g_data_loss_tracker.buffer_samples_dropped,
                g_data_loss_tracker.flash_samples_written,
                samples_in_buffer,
                actual_loss,
                (double)e2e_loss);

        /* Reset period counter */
        g_data_loss_tracker.samples_since_last_report = 0;
        g_data_loss_tracker.last_report_time_ms = now_ms;
    }
}

/*!
 * @brief Track sample successfully stored in ping-pong buffer (Stage 2-3)
 */
static inline void data_loss_tracker_buffer_stored(void)
{
    if (g_data_loss_tracker.tracking_active) {
        g_data_loss_tracker.buffer_samples_stored++;
    }
}

/*!
 * @brief Track sample dropped due to both buffers busy (Stage 2-3)
 */
static inline void data_loss_tracker_buffer_dropped(void)
{
    if (g_data_loss_tracker.tracking_active) {
        g_data_loss_tracker.buffer_samples_dropped++;
        LOG_WRN("DATA_LOSS: Buffer drop! Total dropped: %u", g_data_loss_tracker.buffer_samples_dropped);
    }
}

/*!
 * @brief Track buffer swap event (Stage 2-3)
 */
static inline void data_loss_tracker_buffer_swap(void)
{
    if (g_data_loss_tracker.tracking_active) {
        g_data_loss_tracker.buffer_swap_count++;
    }
}

/*!
 * @brief Track successful flash write (Stage 3-4)
 *
 * @param sample_count Number of samples successfully written
 */
static inline void data_loss_tracker_flash_write_success(uint32_t sample_count)
{
    if (g_data_loss_tracker.tracking_active) {
        g_data_loss_tracker.flash_samples_written += sample_count;
        g_data_loss_tracker.flash_write_count++;
    }
}

/*!
 * @brief Track failed flash write (Stage 3-4)
 *
 * @param sample_count Number of samples that failed to write
 */
static inline void data_loss_tracker_flash_write_failure(uint32_t sample_count)
{
    if (g_data_loss_tracker.tracking_active) {
        g_data_loss_tracker.flash_samples_failed += sample_count;
        g_data_loss_tracker.flash_write_failures++;
        LOG_ERR("DATA_LOSS: Flash write failed! %u samples lost, total failures: %u",
                sample_count, g_data_loss_tracker.flash_write_failures);
    }
}

/*!
 * @brief Get current data loss statistics (end-to-end)
 *
 * @param out_received     Pointer to store total samples received from FIFO
 * @param out_expected     Pointer to store expected samples based on time
 * @param out_lost         Pointer to store total samples lost (all stages)
 * @param out_gap_count    Pointer to store number of FIFO gap events
 * @param out_loss_percent Pointer to store end-to-end loss percentage (0-100)
 */
void ppg_get_data_loss_stats(uint32_t *out_received, uint32_t *out_expected,
                              uint32_t *out_lost, uint32_t *out_gap_count,
                              float *out_loss_percent)
{
    if (out_received) {
        *out_received = g_data_loss_tracker.fifo_samples_received;
    }
    if (out_expected) {
        *out_expected = g_data_loss_tracker.fifo_samples_expected;
    }
    if (out_lost) {
        /* Total lost = FIFO timing loss + buffer drops + flash failures */
        *out_lost = g_data_loss_tracker.fifo_samples_lost +
                   g_data_loss_tracker.buffer_samples_dropped +
                   g_data_loss_tracker.flash_samples_failed;
    }
    if (out_gap_count) {
        *out_gap_count = g_data_loss_tracker.fifo_gap_count;
    }
    if (out_loss_percent) {
        /*
         * True loss = confirmed lost samples / received samples
         * Samples in buffer are pending, not lost
         */
        uint32_t confirmed_loss = g_data_loss_tracker.buffer_samples_dropped +
                                  g_data_loss_tracker.flash_samples_failed;
        if (g_data_loss_tracker.fifo_samples_received > 0) {
            *out_loss_percent = ((float)confirmed_loss /
                                (float)g_data_loss_tracker.fifo_samples_received) * 100.0f;
        } else {
            *out_loss_percent = 0.0f;
        }
    }
}

/*!
 * @brief Get detailed data loss statistics for all stages
 */
void ppg_get_detailed_data_loss_stats(
    uint32_t *out_fifo_received, uint32_t *out_fifo_expected, uint32_t *out_fifo_gaps,
    uint32_t *out_buffer_stored, uint32_t *out_buffer_dropped,
    uint32_t *out_flash_written, uint32_t *out_flash_failed)
{
    if (out_fifo_received) *out_fifo_received = g_data_loss_tracker.fifo_samples_received;
    if (out_fifo_expected) *out_fifo_expected = g_data_loss_tracker.fifo_samples_expected;
    if (out_fifo_gaps) *out_fifo_gaps = g_data_loss_tracker.fifo_gap_count;
    if (out_buffer_stored) *out_buffer_stored = g_data_loss_tracker.buffer_samples_stored;
    if (out_buffer_dropped) *out_buffer_dropped = g_data_loss_tracker.buffer_samples_dropped;
    if (out_flash_written) *out_flash_written = g_data_loss_tracker.flash_samples_written;
    if (out_flash_failed) *out_flash_failed = g_data_loss_tracker.flash_samples_failed;
}

/******************************************************************************
 *                     BMA580 INTERFACE FUNCTIONS                             *
 ******************************************************************************/

/*!
 * @brief Zephyr I2C read function for BMA5 driver
 */
static int8_t bma5_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    int ret;

    if (bma_i2c_dev == NULL) {
        return -1;
    }

    ret = i2c_write_read(bma_i2c_dev, BMA580_I2C_ADDR, &reg_addr, 1, data, len);

    return (ret == 0) ? 0 : -1;
}

/*!
 * @brief Zephyr I2C write function for BMA5 driver
 */
static int8_t bma5_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    int ret;
    uint8_t buf[len + 1];

    if (bma_i2c_dev == NULL) {
        return -1;
    }

    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    ret = i2c_write(bma_i2c_dev, buf, len + 1, BMA580_I2C_ADDR);

    return (ret == 0) ? 0 : -1;
}

/*!
 * @brief Zephyr delay function in microseconds
 */
static void bma5_delay_us(uint32_t period, void *intf_ptr)
{
    k_usleep(period);
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    /* Calculate 2^bit_width / 2 without using pow() */
    float half_scale = (float)(1 << (bit_width - 1));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief Initialize BMA5 interface for Zephyr OS
 */
static int8_t bma5_interface_init(struct bma5_dev *dev, uint8_t intf, enum bma5_context context)
{
    /* Get I2C device from device tree */
    bma_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c21));

    if (!device_is_ready(bma_i2c_dev)) {
        LOG_ERR("BMA580: I2C device not ready - accelerometer will be unavailable");
        bma_i2c_dev = NULL;
        return -1;
    }

    LOG_INF("BMA580: I2C device ready");

    /* Configure the BMA5 device structure */
    dev->intf = intf;
    dev->bus_read = bma5_i2c_read;
    dev->bus_write = bma5_i2c_write;
    dev->delay_us = bma5_delay_us;
    dev->intf_ptr = NULL;
    dev->context = context;
    dev->dummy_byte = 0;

    return 0;
}

/******************************************************************************
 *                           RING BUFFER FUNCTIONS                            *
 ******************************************************************************/

/*! Push a PPG sample to the ring buffer */
static inline void ring_push_ppg(float v)
{
    ppg_ring[ppg_wr] = v;
    ppg_wr = (ppg_wr + 1) % WIN_SAMP_PPG;
    if (ppg_samples_collected < WIN_SAMP_PPG)
        ppg_samples_collected++;
}

/*! Push accelerometer sample to the ring buffer */
static inline void ring_push_acc(float x, float y, float z)
{
    ax_ring[acc_wr] = x;
    ay_ring[acc_wr] = y;
    az_ring[acc_wr] = z;
    acc_wr = (acc_wr + 1) % WIN_SAMP_ACC;
    if (acc_samples_collected < WIN_SAMP_ACC)
        acc_samples_collected++;
}

/*! Load window from ring buffer to workspace (implements overlapping) */
static void load_window_from_ring(ppg_workspace_f_t *w)
{
    /* Load PPG samples - oldest to newest */
    for (int i = 0; i < WIN_SAMP_PPG; i++) {
        int idx = (ppg_wr + i) % WIN_SAMP_PPG;
        w->buf_ppg[i] = ppg_ring[idx];
    }

    /* Load accelerometer samples - oldest to newest */
    for (int i = 0; i < WIN_SAMP_ACC; i++) {
        int idx = (acc_wr + i) % WIN_SAMP_ACC;
        w->acc_x[i] = ax_ring[idx];
        w->acc_y[i] = ay_ring[idx];
        w->acc_z[i] = az_ring[idx];
    }
}

/******************************************************************************
 *                           FLASH HELPER FUNCTIONS                           *
 ******************************************************************************/

#if FLASH_STORAGE_AVAILABLE

/* Pretty-print helper */
static void dump_head_tail_u32(const char *label, const uint32_t *p, size_t count)
{
    LOG_INF("%s (count=%u)", label, (unsigned)count);

    size_t head = (count < DUMP_HEAD_COUNT) ? count : DUMP_HEAD_COUNT;
    size_t tail = (count < DUMP_TAIL_COUNT) ? count : DUMP_TAIL_COUNT;

    if (head) {
        LOG_INF("  head[%u]:", (unsigned)head);
        for (size_t i = 0; i < head; i++) {
            printk("%u ", (unsigned)p[i]);
        }
        printk("\n");
    }

    if (tail && count > head) {
        LOG_INF("  tail[%u]:", (unsigned)tail);
        for (size_t i = count - tail; i < count; i++) {
            printk("%u ", (unsigned)p[i]);
        }
        printk("\n");
    }
}

/* Erase in friendly chunks with progress */
static int erase_region_progress(const struct device *flash, off_t off, size_t bytes)
{
    uint64_t t0 = k_uptime_get();
    size_t erased = 0;

    while (erased < bytes) {
        size_t chunk = bytes - erased;
        if (chunk > ERASE_CHUNK_BYTES) chunk = ERASE_CHUNK_BYTES;

        int rc = flash_erase(flash, off + erased, chunk);
        if (rc) {
            LOG_ERR("ERASE FAIL at 0x%06x (%d)", (unsigned)(off + erased), rc);
            return rc;
        }
        erased += chunk;

        uint64_t now = k_uptime_get();
        LOG_INF("  Erased %u / %u bytes (%.1f%%) in %llums",
               (unsigned)erased, (unsigned)bytes,
               (erased * 100.0) / bytes,
               (unsigned long long)(now - t0));
        k_sleep(K_MSEC(1));
    }
    LOG_INF("Erase complete. Total %u bytes.", (unsigned)bytes);
    return 0;
}

/******************************************************************************
 *                    SECTOR ROTATION HELPER FUNCTIONS                        *
 ******************************************************************************/

/* Calculate XOR checksum for sector header validation */
static uint32_t calc_sector_header_checksum(const sector_header_t *hdr)
{
    return hdr->magic ^ hdr->sequence_num ^ hdr->write_count ^
           hdr->sample_count ^ hdr->timestamp;
}

/* Validate a sector header */
static bool is_valid_sector_header(const sector_header_t *hdr)
{
    if (hdr->magic != SECTOR_HEADER_MAGIC) {
        return false;
    }
    if (hdr->checksum != calc_sector_header_checksum(hdr)) {
        return false;
    }
    return true;
}

/* Get offset for a given sector index */
static inline off_t sector_index_to_offset(uint32_t sector_idx)
{
    return DATA_REGION_START_OFFSET + (sector_idx * FLASH_SECTOR_SIZE);
}

/* Get sector index from offset */
static inline uint32_t offset_to_sector_index(off_t offset)
{
    return (offset - DATA_REGION_START_OFFSET) / FLASH_SECTOR_SIZE;
}

/*
 * Scan all sectors on boot to rebuild wear tracking state and find write head
 *
 * TWO-POINTER SYSTEM:
 * - WRITE HEAD: Found by locating the sector AFTER the highest sequence number
 *   This ensures we continue writing sequentially from where we left off
 * - BLE SYNC: Managed separately in bluetooth_manager.c
 *
 * The write head advances sequentially: 0 -> 1 -> 2 -> ... -> N-1 -> 0 -> ...
 * This guarantees perfectly even wear across all sectors.
 */
static void scan_sectors_and_init(const struct device *flash)
{
    sector_header_t hdr;
    int rc;
    uint32_t highest_seq = 0;
    uint32_t highest_seq_sector = 0;
    uint32_t empty_sectors = 0;
    uint32_t valid_sectors = 0;
    uint32_t corrupted_sectors = 0;
    bool found_any_valid = false;

    LOG_INF("=== Scanning %u sectors for two-pointer state ===", NUM_SECTORS);

    /* Initialize tracking arrays */
    memset(sector_write_counts, 0, sizeof(sector_write_counts));
    memset(sector_valid, 0, sizeof(sector_valid));
    min_write_count = UINT32_MAX;
    max_write_count = 0;
    total_sectors_used = 0;
    global_sequence_num = 0;
    total_flash_samples_written = 0;
    write_cycle_count = 0;

    /* Scan all sectors */
    for (uint32_t i = 0; i < NUM_SECTORS; i++) {
        off_t sector_offset = sector_index_to_offset(i);

        rc = flash_read(flash, sector_offset, &hdr, sizeof(hdr));
        if (rc) {
            LOG_WRN("Flash read fail @ sector %u (0x%06x): %d",
                   i, (unsigned)sector_offset, rc);
            continue;
        }

        /* Check if sector is empty (erased) */
        if (hdr.magic == 0xFFFFFFFF) {
            empty_sectors++;
            sector_write_counts[i] = 0;
            sector_valid[i] = 0;
            continue;
        }

        /* Check if sector has valid header */
        if (is_valid_sector_header(&hdr)) {
            valid_sectors++;
            sector_valid[i] = 1;
            sector_write_counts[i] = (uint16_t)MIN(hdr.write_count, UINT16_MAX);
            total_flash_samples_written += hdr.sample_count;
            found_any_valid = true;

            /* Track min/max write counts */
            if (hdr.write_count < min_write_count) {
                min_write_count = hdr.write_count;
            }
            if (hdr.write_count > max_write_count) {
                max_write_count = hdr.write_count;
            }

            /* Track highest sequence number to find write head position */
            if (hdr.sequence_num > highest_seq) {
                highest_seq = hdr.sequence_num;
                highest_seq_sector = i;
            }
        } else {
            /* Sector has data but invalid header - treat as corrupted/legacy */
            corrupted_sectors++;
            sector_write_counts[i] = 1;  /* Assume at least one write */
            sector_valid[i] = 0;         /* Mark as invalid for overwrite */
        }

        /* Progress logging every 256 sectors */
        if ((i % 256) == 255) {
            LOG_INF("  Scanned %u/%u sectors...", i + 1, NUM_SECTORS);
        }
    }

    /* Set global sequence number to continue from highest found */
    global_sequence_num = highest_seq + 1;
    total_sectors_used = valid_sectors;

    /* Handle case where all sectors were empty */
    if (min_write_count == UINT32_MAX) {
        min_write_count = 0;
    }

    /*
     * DETERMINE WRITE HEAD POSITION (Pointer 1)
     * -----------------------------------------
     * Write head = sector AFTER the last written sector (highest sequence)
     * This ensures sequential circular writing for even wear.
     *
     * Example with 4 sectors:
     * - If last write was to sector 2 (seq=5), write_head = 3
     * - If last write was to sector 3 (seq=5), write_head = 0 (wrap around)
     * - If flash is empty, write_head = 0
     */
    if (found_any_valid) {
        /* Continue from next sector after the last written one */
        write_head_sector = (highest_seq_sector + 1) % NUM_SECTORS;

        /* Calculate write cycle count based on min write count */
        write_cycle_count = min_write_count;

        LOG_INF("=== Write Head Determined ===");
        LOG_INF("  Last written sector: %u (seq=%u)", highest_seq_sector, (unsigned)highest_seq);
        LOG_INF("  Write head (next write): sector %u", write_head_sector);
        LOG_INF("  Complete write cycles: %u", (unsigned)write_cycle_count);
    } else {
        /* Flash is empty - start from sector 0 */
        write_head_sector = 0;
        write_cycle_count = 0;
        LOG_INF("Flash is empty - write head starting at sector 0");
    }

    LOG_INF("=== Sector Scan Complete ===");
    LOG_INF("  Valid sectors: %u", valid_sectors);
    LOG_INF("  Empty sectors: %u", empty_sectors);
    LOG_INF("  Corrupted/Legacy: %u", corrupted_sectors);
    LOG_INF("  Total samples in flash: %u", (unsigned)total_flash_samples_written);
    LOG_INF("  Write count range: %u - %u (diff=%u)",
           (unsigned)min_write_count, (unsigned)max_write_count,
           (unsigned)(max_write_count - min_write_count));
    LOG_INF("  Next sequence number: %u", (unsigned)global_sequence_num);

    sector_tracking_initialized = true;
}

/*
 * SEQUENTIAL WRITE HEAD ADVANCEMENT
 * ----------------------------------
 * This is the core of the two-pointer wear leveling system.
 *
 * Instead of searching for the "best" sector, we simply advance the write head
 * sequentially through all sectors in a circular manner:
 *   0 -> 1 -> 2 -> ... -> (NUM_SECTORS-1) -> 0 -> 1 -> ...
 *
 * This GUARANTEES perfectly even wear:
 * - After N complete cycles, every sector has been written exactly N times
 * - Write counts across sectors differ by at most 1
 * - No complex algorithms or searching required
 *
 * The write_head_sector is determined at boot by scanning for the highest
 * sequence number and setting write_head to the next sector.
 */
static uint32_t advance_write_head(void)
{
    uint32_t sector_to_write = write_head_sector;

    /* Log the write */
    LOG_INF("Sequential write: sector %u (cycle %u, writes=%u)",
           sector_to_write, (unsigned)write_cycle_count,
           sector_write_counts[sector_to_write]);

    /* Advance write head to next sector (circular) */
    write_head_sector = (write_head_sector + 1) % NUM_SECTORS;

    /* Check if we completed a full cycle */
    if (write_head_sector == 0) {
        write_cycle_count++;
        LOG_INF("=== Completed write cycle %u ===", (unsigned)write_cycle_count);
        LOG_INF("  All %u sectors written %u times each", NUM_SECTORS, (unsigned)write_cycle_count);
    }

    /* Mark that write head position has changed and needs to be saved */
    write_head_dirty = true;

    return sector_to_write;
}

/*
 * Legacy compatibility wrapper - now uses sequential advancement
 * This function is called to get the next sector for writing.
 */
static uint32_t find_best_sector_for_write(void)
{
    return advance_write_head();
}

/* Legacy compatibility: wrapper for old code that expects offset-based API */
static off_t find_next_write_offset(const struct device *flash)
{
    /* Initialize sector tracking if not done */
    if (!sector_tracking_initialized) {
        scan_sectors_and_init(flash);
    }

    /* Find best sector using wear leveling */
    current_sector_index = find_best_sector_for_write();
    current_write_offset = sector_index_to_offset(current_sector_index);

    LOG_INF("Next write: sector %u @ offset 0x%06x (write_count=%u)",
           current_sector_index, (unsigned)current_write_offset,
           sector_write_counts[current_sector_index]);

    return current_write_offset;
}

/*
 * Flush PPG RAM buffer to flash with sector header for wear leveling
 * This writes a header at the beginning of the sector containing metadata,
 * followed by the actual PPG sample data.
 */
static int flush_ppg_to_flash(const struct device *flash, off_t off,
                              const uint32_t *buffer, size_t sample_count)
{
    uint32_t sector_idx = offset_to_sector_index(off);
    uint64_t t0 = k_uptime_get();
    int rc;

    /* Limit samples to what fits in sector after header */
    if (sample_count > SAMPLES_PER_SECTOR) {
        LOG_WRN("Sample count %u exceeds sector capacity %u, truncating",
               (unsigned)sample_count, SAMPLES_PER_SECTOR);
        sample_count = SAMPLES_PER_SECTOR;
    }

    uint32_t data_bytes = sample_count * PPG_ELEM_SIZE;

    LOG_INF("Flushing %u samples (%u bytes) to sector %u @ 0x%06x",
           (unsigned)sample_count, (unsigned)data_bytes,
           sector_idx, (unsigned)off);

    /* Prepare sector header */
    sector_header_t hdr = {
        .magic = SECTOR_HEADER_MAGIC,
        .sequence_num = global_sequence_num++,
        .write_count = sector_write_counts[sector_idx] + 1,
        .sample_count = sample_count,
        .timestamp = (uint32_t)k_uptime_get(),
        .reserved = {0, 0}
    };
    hdr.checksum = calc_sector_header_checksum(&hdr);

    LOG_INF("  Header: seq=%u, write_count=%u, samples=%u",
           (unsigned)hdr.sequence_num, (unsigned)hdr.write_count,
           (unsigned)hdr.sample_count);

    /* Erase the sector (required before writing) */
    LOG_INF("  Erasing sector %u...", sector_idx);
    rc = flash_erase(flash, off, FLASH_SECTOR_SIZE);
    if (rc) {
        LOG_ERR("ERASE FAIL at sector %u (0x%06x): %d", sector_idx, (unsigned)off, rc);
        return rc;
    }

    /* Write sector header (first 32 bytes) */
    rc = flash_write(flash, off, &hdr, sizeof(sector_header_t));
    if (rc) {
        LOG_ERR("HEADER WRITE FAIL at 0x%06x (%d)", (unsigned)off, rc);
        return rc;
    }

    /* Write PPG data after header */
    off_t data_offset = off + SECTOR_HEADER_SIZE;
    const uint8_t *src = (const uint8_t *)buffer;
    uint32_t written = 0;

    while (written < data_bytes) {
        uint32_t chunk = MIN(PAGE_SIZE, data_bytes - written);

        /* For the last partial page, we need to pad */
        if (chunk < PAGE_SIZE) {
            uint8_t page_buf[PAGE_SIZE];
            memcpy(page_buf, src + written, chunk);
            memset(page_buf + chunk, 0xFF, PAGE_SIZE - chunk);  /* Pad with 0xFF */
            rc = flash_write(flash, data_offset + written, page_buf, PAGE_SIZE);
        } else {
            rc = flash_write(flash, data_offset + written, src + written, PAGE_SIZE);
        }

        if (rc) {
            LOG_ERR("DATA WRITE FAIL at 0x%06x (%d)", (unsigned)(data_offset + written), rc);
            return rc;
        }
        written += chunk;
    }

    /* Update sector tracking */
    sector_write_counts[sector_idx] = (uint16_t)MIN(hdr.write_count, UINT16_MAX);
    sector_valid[sector_idx] = 1;

    /* Update min/max write counts */
    if (hdr.write_count < min_write_count || min_write_count == 0) {
        min_write_count = hdr.write_count;
    }
    if (hdr.write_count > max_write_count) {
        max_write_count = hdr.write_count;
    }

    uint64_t t1 = k_uptime_get();
    LOG_INF("  Flush complete in %llums. Wear spread: %u-%u writes",
           (unsigned long long)(t1 - t0),
           (unsigned)min_write_count, (unsigned)max_write_count);

    return 0;
}

/*
 * Read back excerpts from flash (accounting for sector header)
 * Data is stored after the SECTOR_HEADER_SIZE byte header
 */
static int read_flash_excerpts_ppg(const struct device *flash, off_t off, size_t sample_count)
{
    /* Data starts after the sector header */
    off_t data_off = off + SECTOR_HEADER_SIZE;

    uint32_t head_buf[DUMP_HEAD_COUNT] = {0};
    size_t head_elems = (sample_count < DUMP_HEAD_COUNT) ? sample_count : DUMP_HEAD_COUNT;
    size_t head_bytes = head_elems * PPG_ELEM_SIZE;
    int rc = flash_read(flash, data_off, head_buf, head_bytes);
    if (rc) {
        LOG_ERR("READ head fail (%d)", rc);
        return rc;
    }

    uint32_t tail_buf[DUMP_TAIL_COUNT] = {0};
    size_t tail_elems = (sample_count < DUMP_TAIL_COUNT) ? sample_count : DUMP_TAIL_COUNT;

    if (sample_count <= head_elems) {
        tail_elems = 0;
    }

    off_t tail_off = data_off + (sample_count - tail_elems) * PPG_ELEM_SIZE;
    rc = flash_read(flash, tail_off, tail_buf, tail_elems * PPG_ELEM_SIZE);
    if (rc) {
        LOG_ERR("READ tail fail (%d)", rc);
        return rc;
    }

    /* Also read and display the sector header info */
    sector_header_t hdr;
    rc = flash_read(flash, off, &hdr, sizeof(hdr));
    if (rc == 0 && is_valid_sector_header(&hdr)) {
        LOG_INF("Sector header: seq=%u, writes=%u, samples=%u, ts=%ums",
               (unsigned)hdr.sequence_num, (unsigned)hdr.write_count,
               (unsigned)hdr.sample_count, (unsigned)hdr.timestamp);
    }

    LOG_INF("FLASH excerpts (count=%u) from @0x%06x (data @0x%06x)",
           (unsigned)sample_count, (unsigned)off, (unsigned)data_off);
    if (head_elems > 0) {
        LOG_INF("  head[%u]:", (unsigned)head_elems);
        for (size_t i = 0; i < head_elems; i++) printk("%u ", (unsigned)head_buf[i]);
        printk("\n");
    }
    if (tail_elems > 0) {
        LOG_INF("  tail[%u]:", (unsigned)tail_elems);
        for (size_t i = 0; i < tail_elems; i++) printk("%u ", (unsigned)tail_buf[i]);
        printk("\n");
    }
    return 0;
}

/*
 * Get wear leveling statistics for external reporting
 */
void ppg_flash_get_wear_stats(uint32_t *out_min_writes, uint32_t *out_max_writes,
                               uint32_t *out_total_sectors, uint32_t *out_valid_sectors)
{
    if (out_min_writes) *out_min_writes = min_write_count;
    if (out_max_writes) *out_max_writes = max_write_count;
    if (out_total_sectors) *out_total_sectors = NUM_SECTORS;
    if (out_valid_sectors) *out_valid_sectors = total_sectors_used;
}

/*
 * Get write head position for BLE status reporting (TWO-POINTER SYSTEM)
 *
 * POINTER 1 (Write Head): Where new data will be written
 * - write_head_sector: Next sector to be written (0 to NUM_SECTORS-1)
 * - write_cycle_count: Number of complete cycles through all sectors
 * - total_samples_written: Total samples written since device initialization
 *
 * POINTER 2 (BLE Sync): Managed in bluetooth_manager.c (transfer_progress)
 */
void ppg_flash_get_write_head(uint32_t *out_write_head_sector,
                               uint32_t *out_write_cycle_count,
                               uint32_t *out_total_samples_written)
{
    if (out_write_head_sector) *out_write_head_sector = write_head_sector;
    if (out_write_cycle_count) *out_write_cycle_count = write_cycle_count;
    if (out_total_samples_written) *out_total_samples_written = total_flash_samples_written;
}

/******************************************************************************
 *                    PING-PONG BUFFER MANAGEMENT FUNCTIONS                   *
 ******************************************************************************/

/*
 * Get buffer state name for logging
 */
static const char* buffer_state_name(ppg_buffer_state_t state)
{
    switch (state) {
        case BUF_STATE_IDLE:      return "IDLE";
        case BUF_STATE_FILLING:   return "FILLING";
        case BUF_STATE_FULL:      return "FULL";
        case BUF_STATE_OFFLOADING: return "OFFLOADING";
        default:                   return "UNKNOWN";
    }
}

/*
 * Initialize ping-pong buffer system
 */
static void pingpong_init(void)
{
    for (int i = 0; i < 2; i++) {
        ppg_buffers[i].sample_count = 0;
        ppg_buffers[i].state = BUF_STATE_IDLE;
        ppg_buffers[i].sector_index = 0;
        ppg_buffers[i].write_offset = 0;
    }

    /* Start with PING buffer as active */
    active_buffer = PPG_BUF_PING;
    ppg_buffers[PPG_BUF_PING].state = BUF_STATE_FILLING;

    buffer_swap_count = 0;
    offload_in_progress_drops = 0;

    LOG_INF("Ping-pong buffer system initialized");
    LOG_INF("  Buffer size: %u samples each (%u bytes)",
           PPG_RAM_BUFFER_SIZE, (unsigned)(PPG_RAM_BUFFER_SIZE * sizeof(uint32_t)));
    LOG_INF("  Total RAM: %u bytes (2 x %u)",
           (unsigned)(2 * PPG_RAM_BUFFER_SIZE * sizeof(uint32_t)),
           (unsigned)(PPG_RAM_BUFFER_SIZE * sizeof(uint32_t)));
}

/*
 * Background work handler for flash offload
 * This runs asynchronously while the other buffer collects data
 */
static void flash_offload_work_handler(struct k_work *work)
{
    /* Find which buffer needs offloading */
    ppg_buffer_id_t offload_buf = (active_buffer == PPG_BUF_PING) ? PPG_BUF_PONG : PPG_BUF_PING;
    ppg_pingpong_buffer_t *buf = &ppg_buffers[offload_buf];

    /* Verify buffer is in correct state */
    if (buf->state != BUF_STATE_OFFLOADING) {
        LOG_WRN("Offload work called but buffer %s is in state %s",
               (offload_buf == PPG_BUF_PING) ? "PING" : "PONG",
               buffer_state_name(buf->state));
        return;
    }

    /* Skip if paused for BLE flash read */
    if (ppg_paused_for_flash_read) {
        LOG_INF("Flash offload deferred - BLE flash read in progress");
        /* Reschedule after a delay on flash workqueue */
        k_work_schedule_for_queue(&flash_workqueue, &flash_offload_work, K_MSEC(100));
        return;
    }

    LOG_INF("=== PING-PONG: Offloading %s buffer ===",
           (offload_buf == PPG_BUF_PING) ? "PING" : "PONG");
    LOG_INF("  Samples: %u", (unsigned)buf->sample_count);
    LOG_INF("  Target sector: %u @ 0x%06x",
           buf->sector_index, (unsigned)buf->write_offset);

    if (buf->sample_count > 0) {
        /* Flush buffer to flash */
        int rc = flush_ppg_to_flash(flash_dev, buf->write_offset,
                                    buf->data, buf->sample_count);

        if (rc == 0) {
            /* Update statistics */
            total_flash_samples_written += buf->sample_count;
            total_sectors_used++;

            /* Track successful flash write (Stage 3-4) */
            data_loss_tracker_flash_write_success(buf->sample_count);

            /* Update BLE flash service */
            uint32_t max_samples_in_flash = NUM_SECTORS * SAMPLES_PER_SECTOR;
            uint32_t samples_available = MIN(total_flash_samples_written, max_samples_in_flash);
            ble_flash_set_params(flash_dev, DATA_REGION_START_OFFSET, samples_available);

            LOG_INF("  Offload SUCCESS: %u total samples in flash",
                   (unsigned)total_flash_samples_written);
        } else {
            /* Track failed flash write (Stage 3-4) */
            data_loss_tracker_flash_write_failure(buf->sample_count);
            LOG_ERR("  Offload FAILED (rc=%d)", rc);
        }
    }

    /* Reset buffer state */
    buf->sample_count = 0;
    buf->state = BUF_STATE_IDLE;

    LOG_INF("  Buffer %s now IDLE and ready",
           (offload_buf == PPG_BUF_PING) ? "PING" : "PONG");

    /* Signal completion */
    k_sem_give(&offload_complete_sem);
}

/*
 * Switch to the other buffer when current one is full
 * Returns true if switch was successful, false if other buffer is still busy
 */
static bool pingpong_switch_buffer(void)
{
    ppg_buffer_id_t current = active_buffer;
    ppg_buffer_id_t other = (current == PPG_BUF_PING) ? PPG_BUF_PONG : PPG_BUF_PING;

    ppg_pingpong_buffer_t *cur_buf = &ppg_buffers[current];
    ppg_pingpong_buffer_t *other_buf = &ppg_buffers[other];

    LOG_INF("=== PING-PONG: Buffer switch requested ===");
    LOG_INF("  Current: %s (%s, %u samples)",
           (current == PPG_BUF_PING) ? "PING" : "PONG",
           buffer_state_name(cur_buf->state),
           (unsigned)cur_buf->sample_count);
    LOG_INF("  Other: %s (%s, %u samples)",
           (other == PPG_BUF_PING) ? "PING" : "PONG",
           buffer_state_name(other_buf->state),
           (unsigned)other_buf->sample_count);

    /* Check if other buffer is available */
    if (other_buf->state != BUF_STATE_IDLE) {
        LOG_WRN("Cannot switch - other buffer busy (%s)", buffer_state_name(other_buf->state));
        offload_in_progress_drops++;
        return false;
    }

    /* Mark current buffer for offload */
    cur_buf->state = BUF_STATE_FULL;

    /* Prepare current buffer for flash write */
    cur_buf->sector_index = current_sector_index;
    cur_buf->write_offset = current_write_offset;

    /* Advance write head for next buffer */
    current_sector_index = find_best_sector_for_write();
    current_write_offset = sector_index_to_offset(current_sector_index);

    /* Switch to other buffer */
    other_buf->state = BUF_STATE_FILLING;
    other_buf->sample_count = 0;
    active_buffer = other;

    /* Start offloading the full buffer */
    cur_buf->state = BUF_STATE_OFFLOADING;

    buffer_swap_count++;

    /* Track buffer swap event (Stage 2-3) */
    data_loss_tracker_buffer_swap();

    LOG_INF("  Switch #%u: Now filling %s, offloading %s",
           (unsigned)buffer_swap_count,
           (other == PPG_BUF_PING) ? "PING" : "PONG",
           (current == PPG_BUF_PING) ? "PING" : "PONG");

    /* Schedule background flash write on dedicated flash workqueue */
    k_work_schedule_for_queue(&flash_workqueue, &flash_offload_work, K_NO_WAIT);

    return true;
}

/*
 * Add a sample to the active buffer
 * Returns true if sample was added, false if buffer is full
 */
static bool pingpong_add_sample(uint32_t sample)
{
    ppg_pingpong_buffer_t *buf = &ppg_buffers[active_buffer];

    /* Check if buffer has space */
    if (buf->sample_count >= PPG_RAM_BUFFER_SIZE) {
        return false;
    }

    /* Add sample */
    buf->data[buf->sample_count++] = sample;

    /* Track successful buffer storage (Stage 2-3) */
    data_loss_tracker_buffer_stored();

    return true;
}

/*
 * Check if active buffer is full
 */
static bool pingpong_is_buffer_full(void)
{
    return ppg_buffers[active_buffer].sample_count >= PPG_RAM_BUFFER_SIZE;
}

/*
 * Get current active buffer's sample count
 */
static size_t pingpong_get_sample_count(void)
{
    return ppg_buffers[active_buffer].sample_count;
}

#else /* !FLASH_STORAGE_AVAILABLE */

/* Stub function when flash storage is disabled */
void ppg_flash_get_wear_stats(uint32_t *out_min_writes, uint32_t *out_max_writes,
                               uint32_t *out_total_sectors, uint32_t *out_valid_sectors)
{
    if (out_min_writes) *out_min_writes = 0;
    if (out_max_writes) *out_max_writes = 0;
    if (out_total_sectors) *out_total_sectors = 0;
    if (out_valid_sectors) *out_valid_sectors = 0;
}

/* Stub function when flash storage is disabled */
void ppg_flash_get_write_head(uint32_t *out_write_head_sector,
                               uint32_t *out_write_cycle_count,
                               uint32_t *out_total_samples_written)
{
    if (out_write_head_sector) *out_write_head_sector = 0;
    if (out_write_cycle_count) *out_write_cycle_count = 0;
    if (out_total_samples_written) *out_total_samples_written = 0;
}

#endif /* FLASH_STORAGE_AVAILABLE */

/******************************************************************************
 *                           BLE FEATURE TRANSMISSION                         *
 ******************************************************************************/

/*! Send HRV features via BLE (separate characteristic in same service)
 *  Format: [SQI(4)] [HR(4)] [SDNN(4)] [RMSSD(4)] [IBI(4)] [Activity(1)]
 *          [BaselineCount(2)] [BaselineTarget(2)] [Z_HR(4)] [STIM(1)]
 *  Total: 32 bytes
 */
static void ble_send_hrv_features(float sqi, FeaturesF *features, int activity,
                                   int baseline_count, int baseline_target,
                                   float z_hr, bool stim)
{
    uint8_t feature_buffer[40];
    uint16_t index = 0;

    /* Pack SQI as float (4 bytes, little-endian) */
    uint32_t sqi_bits;
    memcpy(&sqi_bits, &sqi, sizeof(float));
    feature_buffer[index++] = (sqi_bits) & 0xFF;
    feature_buffer[index++] = (sqi_bits >> 8) & 0xFF;
    feature_buffer[index++] = (sqi_bits >> 16) & 0xFF;
    feature_buffer[index++] = (sqi_bits >> 24) & 0xFF;

    /* Pack HR_mean as float (4 bytes) */
    uint32_t hr_bits;
    memcpy(&hr_bits, &features->HR_mean, sizeof(float));
    feature_buffer[index++] = (hr_bits) & 0xFF;
    feature_buffer[index++] = (hr_bits >> 8) & 0xFF;
    feature_buffer[index++] = (hr_bits >> 16) & 0xFF;
    feature_buffer[index++] = (hr_bits >> 24) & 0xFF;

    /* Pack SDNN as float (4 bytes) */
    uint32_t sdnn_bits;
    memcpy(&sdnn_bits, &features->SDNN, sizeof(float));
    feature_buffer[index++] = (sdnn_bits) & 0xFF;
    feature_buffer[index++] = (sdnn_bits >> 8) & 0xFF;
    feature_buffer[index++] = (sdnn_bits >> 16) & 0xFF;
    feature_buffer[index++] = (sdnn_bits >> 24) & 0xFF;

    /* Pack RMSSD as float (4 bytes) */
    uint32_t rmssd_bits;
    memcpy(&rmssd_bits, &features->RMSSD, sizeof(float));
    feature_buffer[index++] = (rmssd_bits) & 0xFF;
    feature_buffer[index++] = (rmssd_bits >> 8) & 0xFF;
    feature_buffer[index++] = (rmssd_bits >> 16) & 0xFF;
    feature_buffer[index++] = (rmssd_bits >> 24) & 0xFF;

    /* Pack IBI_mean as float (4 bytes) */
    uint32_t ibi_bits;
    memcpy(&ibi_bits, &features->ibi_mean, sizeof(float));
    feature_buffer[index++] = (ibi_bits) & 0xFF;
    feature_buffer[index++] = (ibi_bits >> 8) & 0xFF;
    feature_buffer[index++] = (ibi_bits >> 16) & 0xFF;
    feature_buffer[index++] = (ibi_bits >> 24) & 0xFF;

    /* Pack activity as uint8_t (1 byte) */
    feature_buffer[index++] = (uint8_t)activity;

    /* Pack baseline_count as uint16_t (2 bytes, little-endian) */
    feature_buffer[index++] = (baseline_count) & 0xFF;
    feature_buffer[index++] = (baseline_count >> 8) & 0xFF;

    /* Pack baseline_target as uint16_t (2 bytes, little-endian) */
    feature_buffer[index++] = (baseline_target) & 0xFF;
    feature_buffer[index++] = (baseline_target >> 8) & 0xFF;

    /* Pack z_hr as float (4 bytes) */
    uint32_t z_hr_bits;
    memcpy(&z_hr_bits, &z_hr, sizeof(float));
    feature_buffer[index++] = (z_hr_bits) & 0xFF;
    feature_buffer[index++] = (z_hr_bits >> 8) & 0xFF;
    feature_buffer[index++] = (z_hr_bits >> 16) & 0xFF;
    feature_buffer[index++] = (z_hr_bits >> 24) & 0xFF;

    /* Pack stim decision as uint8_t (1 byte) */
    feature_buffer[index++] = (uint8_t)stim;

    /* Send to HRV Features characteristic (separate from raw PPG data) */
    ble_send_hrv_features_data(feature_buffer, index);

    LOG_DBG("Sent HRV features via BLE: %d bytes", index);
}

/******************************************************************************
 *                       ACCELEROMETER SAMPLING THREAD                        *
 ******************************************************************************/

/*! Real BMA580 accelerometer thread - reads actual sensor data at 10 Hz */
void acc_sampling_thread(void)
{
    const uint32_t acc_period_ms = 1000 / FS_ACC;  /* 100 ms for 10 Hz */
    int8_t rslt;
    uint8_t sensor_ctrl;
    struct bma5_acc_conf acc_cfg;
    struct bma5_accel sens_data;
    struct bma5_sensor_status status;
    enum bma5_context context = BMA5_SMARTPHONE;
    bool sensor_initialized = false;

    LOG_INF("BMA580 ACC thread started at %d ms intervals (%.1f Hz)",
           acc_period_ms, 1000.0f / acc_period_ms);

    /* Initialize BMA580 sensor */
    rslt = bma5_interface_init(&bma_dev, BMA5_I2C_INTF, context);
    if (rslt != BMA5_OK) {
        LOG_WRN("BMA580: Interface init failed (%d) - accelerometer disabled, continuing without it", rslt);
        sensor_initialized = false;
    } else {
        rslt = bma580_init(&bma_dev);
        if (rslt != BMA5_OK) {
            LOG_WRN("BMA580: Sensor init failed (%d) - accelerometer disabled, continuing without it", rslt);
            sensor_initialized = false;
        } else {
            LOG_INF("BMA580: Chip ID: 0x%X", bma_dev.chip_id);

            /* Get current accel configuration */
            rslt = bma5_get_acc_conf(&acc_cfg, &bma_dev);
            if (rslt != BMA5_OK) {
                LOG_WRN("BMA580: Get config failed (%d) - accelerometer disabled, continuing without it", rslt);
                sensor_initialized = false;
            } else {
                /* Set accel configurations for 10 Hz operation */
                acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_25;           /* 25 Hz ODR (closest to 10 Hz) */
                acc_cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;
                acc_cfg.power_mode = BMA5_POWER_MODE_HPM;
                acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_2G;      /* ±2g range */
                acc_cfg.acc_iir_ro = BMA5_ACC_IIR_RO_DB_40;
                acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_POWER;
                acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED;

                rslt = bma5_set_acc_conf(&acc_cfg, &bma_dev);
                if (rslt != BMA5_OK) {
                    LOG_WRN("BMA580: Set config failed (%d) - accelerometer disabled, continuing without it", rslt);
                    sensor_initialized = false;
                } else {
                    /* Enable accelerometer */
                    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;
                    rslt = bma5_set_acc_conf_0(sensor_ctrl, &bma_dev);
                    if (rslt != BMA5_OK) {
                        LOG_WRN("BMA580: Enable sensor failed (%d) - accelerometer disabled, continuing without it", rslt);
                        sensor_initialized = false;
                    } else {
                        sensor_initialized = true;
                        LOG_INF("BMA580: Accelerometer initialized and enabled successfully");
                        LOG_INF("BMA580: Range=±2G, ODR=25Hz, Mode=HPM");
                    }
                }
            }
        }
    }

    /* Main sampling loop */
    while (1) {
        if (sensor_initialized) {
            /* Get sensor status to check if data is ready */
            rslt = bma5_get_sensor_status(&status, &bma_dev);
            if (rslt == BMA5_OK) {
                /* Read accelerometer data */
                rslt = bma5_get_acc(&sens_data, &bma_dev);

                if (rslt == BMA5_OK) {
                    /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
                    float x_ms2 = lsb_to_ms2(sens_data.x, 2.0f, BMA5_16_BIT_RESOLUTION);
                    float y_ms2 = lsb_to_ms2(sens_data.y, 2.0f, BMA5_16_BIT_RESOLUTION);
                    float z_ms2 = lsb_to_ms2(sens_data.z, 2.0f, BMA5_16_BIT_RESOLUTION);

                    /* Convert from m/s² to mg (milligrams) for HRM algorithm */
                    /* 1g = 9.80665 m/s² = 1000 mg */
                    /* mg = (m/s²) * (1000 / 9.80665) */
                    float x_mg = x_ms2 * (1000.0f / GRAVITY_EARTH);
                    float y_mg = y_ms2 * (1000.0f / GRAVITY_EARTH);
                    float z_mg = z_ms2 * (1000.0f / GRAVITY_EARTH);

                    /* Push real accelerometer data to ring buffer */
                    ring_push_acc(x_mg, y_mg, z_mg);
                } else {
                    LOG_WRN("BMA580: Read data failed (%d)", rslt);
                }
            }
        } else {
            /* Sensor not initialized - log error only once every 100 iterations to reduce log spam */
            static uint32_t error_count = 0;
            if ((error_count % 100) == 0) {
                LOG_WRN("BMA580: Sensor not initialized - no accelerometer data available (count: %u)", error_count);
            }
            error_count++;
        }

        /* Sleep until next sample */
        k_sleep(K_MSEC(acc_period_ms));
    }
}

/*! Callback that is called by the Chip Library when new data is available, see ::as7058_callback_t. */
static void as7058_callback(err_code_t error, const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                            const agc_status_t *p_agc_statuses, uint8_t agc_statuses_num,
                            as7058_status_events_t sensor_events, const void *p_cb_param)
{
    int sub_sample_id_index;
    err_code_t result;
    double sub_sample_period;
    uint32_t samples[48];
    uint16_t sample_cnt;
    static uint8_t ppg_buffer[192];  // Buffer for BLE: 48 samples × 4 bytes each

    /* Mark parameter as unused to silence warnings. */
    M_UNUSED_PARAM(p_cb_param);
    M_UNUSED_PARAM(p_agc_statuses);
    M_UNUSED_PARAM(agc_statuses_num);
    M_UNUSED_PARAM(sensor_events);

    /* Check if error occurred in the Chip Library. */
    if (error != ERR_SUCCESS) {
        LOG_ERR("Received error code %d from Chip Library", error);
        return;
    }

    /* Extract and process samples for PPG1_SUB1. */
    sub_sample_id_index = AS7058_SUB_SAMPLE_ID_PPG1_SUB1;
    sub_sample_period = g_ppg_sample_period_s;

    /* Call the FIFO data extract function, which copies all samples of a given sub-sample to a signed 32-bit array. */
    sample_cnt = sizeof(samples) / sizeof(samples[0]);
    result = as7058_extract_samples(sub_sample_id_index, p_fifo_data, fifo_data_size, samples, &sample_cnt,
                                    (as7058_extract_metadata_t *)&g_extract_metadata);

    if (result == ERR_SUCCESS && sample_cnt > 0) {
#if FLASH_STORAGE_AVAILABLE
        /* Skip all processing if paused for flash read */
        if (ppg_paused_for_flash_read) {
            /* Silently drop samples during flash read to avoid BLE contention */
            return;
        }
#endif

        /* Track data loss - Stage 1-2: FIFO to callback */
        data_loss_tracker_process_fifo_samples(sample_cnt);

        /* Push each PPG sample to the ring buffer for HRM processing */
        for (int i = 0; i < sample_cnt; i++) {
            ring_push_ppg((float)samples[i]);
        }

#if FLASH_STORAGE_AVAILABLE
        /*
         * PING-PONG BUFFER: Store PPG data with automatic buffer swapping
         *
         * When active buffer fills:
         * 1. Switch to other buffer (continues collecting without pause)
         * 2. Trigger background flash write for full buffer
         * 3. No data loss during flash writes!
         */
        if (ppg_flash_storage_enabled) {
            for (int i = 0; i < sample_cnt; i++) {
                /* Try to add sample to active buffer */
                if (!pingpong_add_sample((uint32_t)samples[i])) {
                    /* Buffer is full - try to switch */
                    if (pingpong_switch_buffer()) {
                        /* Switch successful - add sample to new buffer */
                        pingpong_add_sample((uint32_t)samples[i]);
                    } else {
                        /* Both buffers busy - sample dropped */
                        data_loss_tracker_buffer_dropped();
                    }
                }

                /* Print progress every 256 samples */
                size_t count = pingpong_get_sample_count();
                if ((count % 256) == 0 && count > 0) {
                    LOG_INF("Ping-pong [%s]: %u / %u samples",
                           (active_buffer == PPG_BUF_PING) ? "PING" : "PONG",
                           (unsigned)count, (unsigned)PPG_RAM_BUFFER_SIZE);
                }
            }
        }
#endif

        /* Pack ALL samples into BLE buffer (4 bytes per sample, big-endian) */
        uint16_t buffer_index = 0;
        for (int i = 0; i < sample_cnt && buffer_index < sizeof(ppg_buffer); i++) {
            int32_t sample = (int32_t)samples[i];
            ppg_buffer[buffer_index++] = (sample >> 24) & 0xFF;
            ppg_buffer[buffer_index++] = (sample >> 16) & 0xFF;
            ppg_buffer[buffer_index++] = (sample >> 8) & 0xFF;
            ppg_buffer[buffer_index++] = sample & 0xFF;
        }

        /* Send notification to Raw PPG Data characteristic */
        ble_send_ppg_data(ppg_buffer, buffer_index);

        g_sub_sample_cnt[sub_sample_id_index] += sample_cnt;
    }
}

static err_code_t ppg_configure_registers(void)
{
    err_code_t result = ERR_SUCCESS;
    LOG_INF("Configuring AS7058 registers for PPG...");

    /**************************************************************************
     *                           CHIP CONFIGURATION                           *
     **************************************************************************
     * This section uses the Chip Library to apply a configuration to the     *
     * AS7058 device that enables PPG measurement.                            *
     **************************************************************************/

    /* Configure register group POWER. */
    const as7058_reg_group_power_t power_config = {{
        .pwr_on = 31,
        .pwr_iso = 0,
        .clk_cfg = 7,
        .ref_cfg1 = 63,
        .ref_cfg2 = 14,
        .ref_cfg3 = 160,
        .standby_on1 = 0,
        .standby_on2 = 0,
        .standby_en1 = 4,
        .standby_en2 = 2,
        .standby_en3 = 4,
        .standby_en4 = 0,
        .standby_en5 = 3,
        .standby_en6 = 16,
        .standby_en7 = 16,
        .standby_en8 = 4,
        .standby_en9 = 0,
        .standby_en10 = 3,
        .standby_en11 = 16,
        .standby_en12 = 16,
        .standby_en13 = 16,
        .standby_en14 = 16,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_PWR, power_config.reg_buffer, sizeof(as7058_reg_group_power_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_PWR returned error %d", result);
        return result;
    }

    /* Configure register group CONTROL. */
    const as7058_reg_group_control_t control_config = {{
        .i2c_mode = 0,
        .int_cfg = 0,
        .if_cfg = 72,
        .gpio_cfg1 = 0,
        .gpio_cfg2 = 0,
        .io_cfg = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_CTRL, control_config.reg_buffer, sizeof(as7058_reg_group_control_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_CTRL returned error %d", result);
        return result;
    }

    /* Configure register group LED. */
    const as7058_reg_group_led_t led_config = {{
        .vcsel_password = 87,
        .vcsel_cfg = 192,
        .vcsel_mode = 0,
        .led_cfg = 1,
        .led_drv1 = 0,
        .led_drv2 = 0,
        .led1_ictrl = 9,
        .led2_ictrl = 0,
        .led3_ictrl = 0,
        .led4_ictrl = 0,
        .led5_ictrl = 0,
        .led6_ictrl = 0,
        .led7_ictrl = 0,
        .led8_ictrl = 0,
        .led_irng1 = 63,
        .led_irng2 = 0,
        .led_sub1 = 1,
        .led_sub2 = 0,
        .led_sub3 = 0,
        .led_sub4 = 0,
        .led_sub5 = 0,
        .led_sub6 = 0,
        .led_sub7 = 0,
        .led_sub8 = 0,
        .lowvds_wait = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_LED, led_config.reg_buffer, sizeof(as7058_reg_group_led_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_LED returned error %d", result);
        return result;
    }

    /* Configure register group PD. */
    const as7058_reg_group_pd_t pd_config = {{
        .pdsel_cfg = 0,
        .ppg1_pdsel1 = 2,
        .ppg1_pdsel2 = 0,
        .ppg1_pdsel3 = 0,
        .ppg1_pdsel4 = 0,
        .ppg1_pdsel5 = 0,
        .ppg1_pdsel6 = 7,
        .ppg1_pdsel7 = 0,
        .ppg1_pdsel8 = 0,
        .ppg2_pdsel1 = 0,
        .ppg2_pdsel2 = 0,
        .ppg2_pdsel3 = 0,
        .ppg2_pdsel4 = 0,
        .ppg2_pdsel5 = 0,
        .ppg2_pdsel6 = 1,
        .ppg2_pdsel7 = 0,
        .ppg2_pdsel8 = 1,
        .ppg2_afesel1 = 0,
        .ppg2_afesel2 = 0,
        .ppg2_afesel3 = 0,
        .ppg2_afesel4 = 0,
        .ppg2_afeen = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_PD, pd_config.reg_buffer, sizeof(as7058_reg_group_pd_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_PD returned error %d", result);
        return result;
    }

    /* Configure register group IOS. */
    const as7058_reg_group_ios_t ios_config = {{
        .ios_ppg1_sub1 = 72,
        .ios_ppg1_sub2 = 0,
        .ios_ppg1_sub3 = 0,
        .ios_ppg1_sub4 = 0,
        .ios_ppg1_sub5 = 0,
        .ios_ppg1_sub6 = 0,
        .ios_ppg1_sub7 = 0,
        .ios_ppg1_sub8 = 0,
        .ios_ppg2_sub1 = 0,
        .ios_ppg2_sub2 = 0,
        .ios_ppg2_sub3 = 0,
        .ios_ppg2_sub4 = 0,
        .ios_ppg2_sub5 = 0,
        .ios_ppg2_sub6 = 0,
        .ios_ppg2_sub7 = 0,
        .ios_ppg2_sub8 = 0,
        .ios_ledoff = 0,
        .ios_cfg = 0,
        .aoc_sar_thres = 0,
        .aoc_sar_range = 0,
        .aoc_sar_ppg1 = 0,
        .aoc_sar_ppg2 = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_IOS, ios_config.reg_buffer, sizeof(as7058_reg_group_ios_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_IOS returned error %d", result);
        return result;
    }

    /* Configure register group PPG. */
    const as7058_reg_group_ppg_t ppg_config = {{
        .ppgmod_cfg1 = 0,
        .ppgmod_cfg2 = 0,
        .ppgmod_cfg3 = 0,
        .ppgmod1_cfg1 = 135,
        .ppgmod1_cfg2 = 84,
        .ppgmod1_cfg3 = 7,
        .ppgmod2_cfg1 = 7,
        .ppgmod2_cfg2 = 84,
        .ppgmod2_cfg3 = 7,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_PPG, ppg_config.reg_buffer, sizeof(as7058_reg_group_ppg_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_PPG returned error %d", result);
        return result;
    }

    /* Configure register group ECG. */
    const as7058_reg_group_ecg_t ecg_config = {{
        .bioz_cfg = 0,
        .bioz_excit = 0,
        .bioz_mixer = 0,
        .bioz_select = 0,
        .bioz_gain = 0,
        .ecgmod_cfg1 = 0,
        .ecgmod_cfg2 = 0,
        .ecgimux_cfg1 = 0,
        .ecgimux_cfg2 = 0,
        .ecgimux_cfg3 = 0,
        .ecgamp_cfg1 = 0,
        .ecgamp_cfg2 = 0,
        .ecgamp_cfg3 = 0,
        .ecgamp_cfg4 = 0,
        .ecgamp_cfg5 = 0,
        .ecgamp_cfg6 = 0,
        .ecgamp_cfg7 = 0,
        .ecg_bioz = 0,
        .leadoff_cfg = 0,
        .leadoff_thresl = 0,
        .leadoff_thresh = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_ECG, ecg_config.reg_buffer, sizeof(as7058_reg_group_ecg_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_ECG returned error %d", result);
        return result;
    }

    /* Configure register group SINC. */
    const as7058_reg_group_sinc_t sinc_config = {{
        .ppg_sinc_cfga = 2,
        .ppg_sinc_cfgb = 3,
        .ppg_sinc_cfgc = 0,
        .ppg_sinc_cfgd = 0,
        .ecg1_sinc_cfga = 0,
        .ecg1_sinc_cfgb = 0,
        .ecg1_sinc_cfgc = 0,
        .ecg2_sinc_cfga = 0,
        .ecg2_sinc_cfgb = 0,
        .ecg2_sinc_cfgc = 0,
        .ecg_sinc_cfg = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_SINC, sinc_config.reg_buffer, sizeof(as7058_reg_group_sinc_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_SINC returned error %d", result);
        return result;
    }

    /* Configure register group SEQ. */
    const as7058_reg_group_seq_t seq_config = {{
        .irq_enable = 255,
        .ppg_sub_wait = 0,
        .ppg_sar_wait = 0,
        .ppg_led_init = 10,
        .ppg_freql = 31,
        .ppg_freqh = 3,
        .ppg1_sub_en = 1,
        .ppg2_sub_en = 0,
        .ppg_mode_1 = 0,
        .ppg_mode_2 = 0,
        .ppg_mode_3 = 0,
        .ppg_mode_4 = 0,
        .ppg_mode_5 = 0,
        .ppg_mode_6 = 0,
        .ppg_mode_7 = 0,
        .ppg_mode_8 = 0,
        .ppg_cfg = 6,
        .ecg_freql = 0,
        .ecg_freqh = 0,
        .ecg1_freqdivl = 0,
        .ecg1_freqdivh = 0,
        .ecg2_freqdivl = 0,
        .ecg2_freqdivh = 0,
        .ecg_subs = 0,
        .leadoff_initl = 0,
        .leadoff_inith = 0,
        .ecg_initl = 0,
        .ecg_inith = 0,
        .sample_num = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_SEQ, seq_config.reg_buffer, sizeof(as7058_reg_group_seq_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_SEQ returned error %d", result);
        return result;
    }

    /* Configure register group PP. */
    const as7058_reg_group_pp_t post_config = {{
        .pp_cfg = 0,
        .ppg1_pp1 = 0,
        .ppg1_pp2 = 0,
        .ppg2_pp1 = 0,
        .ppg2_pp2 = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_PP, post_config.reg_buffer, sizeof(as7058_reg_group_pp_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_PP returned error %d", result);
        return result;
    }

    /* Configure register group FIFO. */
    const as7058_reg_group_fifo_t fifo_config = {{
        .fifo_threshold = 39,
        .fifo_ctrl = 0,
    }};
    result = as7058_set_reg_group(AS7058_REG_GROUP_ID_FIFO, fifo_config.reg_buffer, sizeof(as7058_reg_group_fifo_t));
    if (result != ERR_SUCCESS) {
        LOG_ERR("Writing register group AS7058_REG_GROUP_ID_FIFO returned error %d", result);
        return result;
    }

    /**************************************************************************
     *                     AGC CONFIGURATION                                  *
     **************************************************************************/

    /* Configure automatic gain control (AGC) for LED current optimization. */
    const agc_configuration_t agc_config = {
        .mode = AGC_MODE_DEFAULT,
        .led_control_mode = AGC_AMPL_CNTL_MODE_AUTO,
        .channel = AS7058_SUB_SAMPLE_ID_PPG1_SUB1,
        .led_current_min = 5,
        .led_current_max = 30,
        .rel_amplitude_min_x100 = 5,
        .rel_amplitude_max_x100 = 20,
        .rel_amplitude_motion_x100 = 50,
        .num_led_steps = 1,
        .reserved = {0, 0, 0},
        .threshold_min = 300000,
        .threshold_max = 800000,
    };
    result = as7058_set_agc_config(&agc_config, 1);
    if (result != ERR_SUCCESS) {
        LOG_ERR("as7058_set_agc_config returned error code %d", result);
        return result;
    }
    LOG_INF("AGC configured successfully for PPG1_SUB1");

    LOG_INF("Register configuration complete.");
    return result;
}

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

int ppg_thread_main(void)
{
    ppg_cmd_t cmd;
    static ppg_state_t state = PPG_STATE_OFF;
    err_code_t result;
    bool hrm_buffers_filled = false;

    LOG_INF("PPG thread started");

      /* init baseline state */
    stim_state_init_f(&ws);

#if FLASH_STORAGE_AVAILABLE
    /**************************************************************************
     *                     FLASH INITIALIZATION WITH WEAR LEVELING            *
     **************************************************************************/
    /* Initialize the Flash Device */
    flash_dev = DEVICE_DT_GET_ONE(FLASH_COMPAT);
    if (!device_is_ready(flash_dev)) {
        LOG_WRN("Flash device not ready - flash storage disabled");
        flash_dev = NULL;
    } else {
        LOG_INF("=== PPG Flash Storage with Wear Leveling ===");
        LOG_INF("Flash Device: %s", flash_dev->name);
        LOG_INF("Sector size: %u bytes", FLASH_SECTOR_SIZE);
        LOG_INF("Sector header: %u bytes", SECTOR_HEADER_SIZE);
        LOG_INF("Data per sector: %u samples (%u bytes)",
               SAMPLES_PER_SECTOR, SECTOR_DATA_CAPACITY);
        LOG_INF("Total sectors: %u (%.2f MB)",
               NUM_SECTORS, (double)DATA_REGION_TOTAL_SIZE / (1024.0 * 1024.0));
        LOG_INF("Total capacity: %u samples",
               (unsigned)(NUM_SECTORS * SAMPLES_PER_SECTOR));

        /*
         * Scan all sectors to rebuild wear leveling state
         * This reads headers to find:
         * - Valid/invalid/empty sectors
         * - Write counts for each sector
         * - Total samples stored
         * - Next sequence number
         */
        current_write_offset = find_next_write_offset(flash_dev);

        /* Log wear leveling status */
        LOG_INF("=== Wear Leveling Status ===");
        LOG_INF("  Sectors with data: %u / %u (%.1f%%)",
               (unsigned)total_sectors_used, NUM_SECTORS,
               (total_sectors_used * 100.0f) / NUM_SECTORS);
        LOG_INF("  Total samples in flash: %u", (unsigned)total_flash_samples_written);
        LOG_INF("  Write count spread: %u - %u (diff=%u)",
               (unsigned)min_write_count, (unsigned)max_write_count,
               (unsigned)(max_write_count - min_write_count));
        LOG_INF("  Wear threshold: %u writes", WEAR_LEVEL_THRESHOLD);
        LOG_INF("  Next write: sector %u @ 0x%06x",
               current_sector_index, (unsigned)current_write_offset);

        /*
         * Initialize BLE flash service with existing data
         * This allows reading flash data via BLE without starting PPG measurement
         */
        if (total_flash_samples_written > 0) {
            uint32_t max_samples_in_flash = NUM_SECTORS * SAMPLES_PER_SECTOR;
            uint32_t samples_available = MIN(total_flash_samples_written, max_samples_in_flash);
            LOG_INF("=== Initializing BLE Flash Access ===");
            LOG_INF("  Setting up BLE to read %u samples from flash", (unsigned)samples_available);
            ble_flash_set_params(flash_dev, DATA_REGION_START_OFFSET, samples_available);
            LOG_INF("  BLE flash read available immediately (no PPG start required)");
        } else {
            LOG_INF("No existing data in flash - BLE flash read will be available after first write");
        }

        /*
         * Initialize dedicated flash workqueue
         * Isolates flash I/O (50-100ms) from system workqueue to prevent BLE starvation
         */
        LOG_INF("=== Initializing Flash Workqueue ===");
        k_work_queue_start(&flash_workqueue, flash_workqueue_stack,
                          K_THREAD_STACK_SIZEOF(flash_workqueue_stack),
                          FLASH_WORKQUEUE_PRIORITY, NULL);
        k_thread_name_set(&flash_workqueue.thread, "flash_wq");

        /*
         * Initialize ping-pong double buffer system
         * This enables continuous data acquisition without gaps during flash writes
         */
        LOG_INF("=== Initializing Ping-Pong Buffer System ===");
        k_work_init_delayable(&flash_offload_work, flash_offload_work_handler);
        pingpong_init();

        ppg_flash_storage_enabled = true;
        LOG_INF("Flash storage ready with wear leveling and ping-pong buffers enabled");
    }
#endif

    // This thread loops forever, waiting for messages or processing HRM
    while (1) {
        // Wait for a command with timeout (STEP_SEC) to allow periodic HRM processing
        int ret = k_msgq_get(&ppg_cmd_q, &cmd, K_SECONDS(STEP_SEC));

        // If timeout occurred and we're measuring, process HRM features
        if (ret == -EAGAIN && state == PPG_STATE_MEASURING) {
#if FLASH_STORAGE_AVAILABLE
            /* Skip HRM processing if paused for flash read */
            if (ppg_paused_for_flash_read) {
                continue;
            }
#endif

            // Check if buffers are filled
            if (!hrm_buffers_filled) {
                if (ppg_samples_collected >= WIN_SAMP_PPG &&
                    acc_samples_collected >= WIN_SAMP_ACC) {
                    hrm_buffers_filled = true;
                    LOG_INF("===========================================");
                    LOG_INF("HRM pipeline started!");
                    LOG_INF("Window: %d s, Step: %d s", WIN_SEC, STEP_SEC);
                    LOG_INF("===========================================");
                } else {
                    // Still filling buffers, log progress occasionally
                    static int progress_counter = 0;
                    if (progress_counter % 3 == 0) {
                        LOG_INF("Filling buffers: PPG=%d/%d, ACC=%d/%d",
                               ppg_samples_collected, WIN_SAMP_PPG,
                               acc_samples_collected, WIN_SAMP_ACC);
                    }
                    progress_counter++;
                    continue;
                }
            }

            // Process HRM features if buffers are filled
            if (hrm_buffers_filled) {
                load_window_from_ring(&ws);

                // Run HRM processing pipeline
                acc_prepare_vm_f(&ws);
                preprocess_ppg_f(&ws);

                /* ---------- FLATLINE CHECK (after preprocessing) ---------- */

        float seg_max = custom_max_f(ws.buf_filt, WIN_SAMP_PPG);
        float seg_min = custom_min_f(ws.buf_filt, WIN_SAMP_PPG);
        float seg_range = seg_max - seg_min;

        /* median */
        float seg_median = custom_median_f_ws(ws.buf_filt,
                                            WIN_SAMP_PPG,
                                            ws.scratch1);

        /* MAD = median(|x - median|) */
        for (int i = 0; i < WIN_SAMP_PPG; i++) {
            ws.scratch2[i] = fabsf(ws.buf_filt[i] - seg_median);
        }
        float seg_mad = custom_median_f_ws(ws.scratch2,
                                        WIN_SAMP_PPG,
                                        ws.scratch1);

        /* thresholds */
        float abs_med = fabsf(seg_median);
        float flatline_thresh1 = 1e-3f * fmaxf(1.0f, abs_med);
        float flatline_thresh2 = 1e-6f * fmaxf(1.0f, abs_med);

        /* flatline decision */
        int flatline = (seg_range < flatline_thresh1) ||
                    (seg_mad   < flatline_thresh2);

        /* ---------- SQI + FEATURE LOGIC ---------- */

        float sqi;

        if (flatline) {

            sqi = 0.0f;
            make_nan_features_f_ws(&ws);

        } else {

            sqi = SQI_PPG_f(&ws);

            if (sqi > 0.8f) {
                feature_extraction_PPG_f(&ws);
            } else {
                make_nan_features_f_ws(&ws);
            }
        }

        int activity = Acc_activity_f(&ws);

        /* ---- BASELINE UPDATE (only until ready) ---- */
        stim_baseline_update_f(&ws, sqi, activity, ws.features.HR_mean);

        /* ---- DECISION (only after baseline ready) ---- */
        stim_decide_f(&ws, sqi, activity, ws.features.HR_mean);

        printk("BASELINE: %d/%d  SQI=%.2f HR=%.1f SDNN=%.2f RMSSD=%.2f IBI=%.1f ACT=%d zHR=%.2f DEC=%s\n",
               ws.stim_state.baseline_count,
               ws.stim_state.baseline_target,
               (double)sqi,
               (double)ws.features.HR_mean,
               (double)ws.features.SDNN,
               (double)ws.features.RMSSD,
               (double)ws.features.ibi_mean,
               activity,
               (double)ws.stim_state.z_hr,
               ws.stim_state.stim ? "STIM" : "NO STIM");


                // Send features via BLE
                ble_send_hrv_features(sqi, &ws.features, activity,
                                      ws.stim_state.baseline_count,
                                      ws.stim_state.baseline_target,
                                      ws.stim_state.z_hr,
                                      ws.stim_state.stim);
            }

#if FLASH_STORAGE_AVAILABLE
            /*
             * PING-PONG STATUS: Log periodic stats (no manual flush needed)
             *
             * The ping-pong system handles all flushing automatically via
             * background work queue. This just logs status periodically.
             */
            static uint32_t last_status_log = 0;
            uint32_t now = k_uptime_get_32();
            if ((now - last_status_log) >= 30000) {  /* Log every 30 seconds */
                last_status_log = now;
                LOG_INF("=== Ping-Pong Status ===");
                LOG_INF("  Active buffer: %s (%u/%u samples)",
                       (active_buffer == PPG_BUF_PING) ? "PING" : "PONG",
                       (unsigned)pingpong_get_sample_count(),
                       (unsigned)PPG_RAM_BUFFER_SIZE);
                LOG_INF("  PING state: %s", buffer_state_name(ppg_buffers[PPG_BUF_PING].state));
                LOG_INF("  PONG state: %s", buffer_state_name(ppg_buffers[PPG_BUF_PONG].state));
                LOG_INF("  Buffer swaps: %u", (unsigned)buffer_swap_count);
                LOG_INF("  Dropped samples: %u", (unsigned)offload_in_progress_drops);
                LOG_INF("  Total samples in flash: %u", (unsigned)total_flash_samples_written);
                LOG_INF("  Write head: sector %u (cycle %u)",
                       write_head_sector, (unsigned)write_cycle_count);
            }
#endif

            continue;  // Go back to waiting for next command/timeout
        }

        // If we got a command (not timeout), process it
        if (ret == 0) {
            switch (cmd) {

            case PPG_CMD_BLE_START:
                if (state == PPG_STATE_OFF) {
                    LOG_INF("START command received. Initializing PPG sensor.");

                    // Reset HRM buffers and counters
                    ppg_samples_collected = 0;
                    acc_samples_collected = 0;
                    ppg_wr = 0;
                    acc_wr = 0;
                    hrm_buffers_filled = false;

#if FLASH_STORAGE_AVAILABLE
                    /*
                     * Reset ping-pong buffers (but keep total_flash_samples_written!)
                     * NOTE: total_flash_samples_written is NOT reset - it persists across PPG start/stop
                     * This ensures BLE can always read ALL data in flash, not just new data
                     */
                    pingpong_init();
                    buffer_swap_count = 0;
                    offload_in_progress_drops = 0;
                    LOG_INF("Ping-pong buffers reset (total samples preserved: %u)",
                           (unsigned)total_flash_samples_written);
#endif

                    // --- 1. INITIALIZE ---
                    result = as7058_initialize(as7058_callback, NULL, NULL, NULL);
                    if (result != ERR_SUCCESS) {
                        LOG_ERR("AS7058: Initialize failed: %d - PPG sensor not available, staying in OFF state", result);
                        LOG_ERR("AS7058: System will continue without PPG functionality");
                        state = PPG_STATE_OFF;
                        continue;
                    }

                    // --- 2. CONFIGURE ---
                    result = ppg_configure_registers();
                    if (result != ERR_SUCCESS) {
                        LOG_ERR("AS7058: Configure registers failed: %d - PPG sensor not available", result);
                        LOG_ERR("AS7058: System will continue without PPG functionality");
                        state = PPG_STATE_OFF;
                        as7058_shutdown();
                        continue;
                    }

                    // --- 3. GET MEASUREMENT CONFIG ---
                    as7058_meas_config_t meas_config;
                    result = as7058_get_measurement_config(&meas_config);
                    if (result != ERR_SUCCESS) {
                        LOG_ERR("AS7058: Get measurement config failed: %d - PPG sensor not available", result);
                        LOG_ERR("AS7058: System will continue without PPG functionality");
                        state = PPG_STATE_OFF;
                        as7058_shutdown();
                        continue;
                    }

                    // Initialize extract metadata
                    g_extract_metadata.copy_recent_to_current = FALSE;
                    g_extract_metadata.fifo_map = meas_config.fifo_map;
                    g_extract_metadata.current.ppg1_sub = 0;
                    g_extract_metadata.current.ppg2_sub = 0;
                    g_extract_metadata.recent.ppg1_sub = 0;
                    g_extract_metadata.recent.ppg2_sub = 0;

                    // Save the sample period
                    g_ppg_sample_period_s = (double)meas_config.ppg_sample_period_us / 1000 / 1000;

                    // Reset sample count
                    for (int i = 0; i < AS7058_SUB_SAMPLE_ID_NUM; i++) {
                        g_sub_sample_cnt[i] = 0;
                    }

                    // --- 4. START MEASUREMENT ---
                    result = as7058_start_measurement(AS7058_MEAS_MODE_NORMAL);
                    if (result != ERR_SUCCESS) {
                        LOG_ERR("AS7058: Start measurement failed: %d - PPG sensor not available", result);
                        LOG_ERR("AS7058: System will continue without PPG functionality");
                        state = PPG_STATE_OFF;
                        as7058_shutdown();
                    } else {
                        LOG_INF("*** AS7058: PPG measurement started successfully");
                        LOG_INF("HRM processing will start after %d seconds", WIN_SEC);
                        state = PPG_STATE_MEASURING;

                        /* Start data loss tracking for 40Hz validation */
                        data_loss_tracker_start();
                    }
                } else {
                    LOG_WRN("START command received but state is not OFF (%d)", state);
                }
                break;

            case PPG_CMD_BLE_STOP:
                if (state == PPG_STATE_MEASURING) {
                    LOG_INF("STOP command received. Shutting down PPG sensor.");

                    /* Stop data loss tracking and log final statistics */
                    data_loss_tracker_stop();

                    // --- STOP AND SHUTDOWN ---
                    result = as7058_stop_measurement();
                    if (result != ERR_SUCCESS) {
                        LOG_ERR("stop_measurement failed: %d", result);
                    }

                    (void)as7058_shutdown();

                    state = PPG_STATE_OFF;
                    hrm_buffers_filled = false;
                    LOG_INF("HRM processing stopped.");
                } else {
                    LOG_WRN("STOP command received but sensor already off.");
                }
                break;

            case PPG_CMD_INTERNAL_FIFO_READY:
                // Data is already processed in the callback
                // This command is not used in PPG mode but kept for potential future use
                break;
            } // end switch
        } // end if (ret == 0)
    } // end while(1)
}

/******************************************************************************
 *                        FLASH READ PAUSE/RESUME API                         *
 ******************************************************************************/

/**
 * @brief Pause PPG processing during flash read operations
 *
 * Call this before starting a large flash read to prevent:
 * - PPG data collection (flash buffer writes)
 * - HRM algorithm processing
 * - Real-time BLE streaming
 * This allows flash read to complete faster without BLE contention.
 */
void ppg_pause_for_flash_read(void)
{
#if FLASH_STORAGE_AVAILABLE
    ppg_paused_for_flash_read = true;
    LOG_INF("PPG processing PAUSED for flash read operation");
#endif
}

/**
 * @brief Resume PPG processing after flash read completes
 *
 * Call this after flash read completes to resume normal operation.
 */
void ppg_resume_after_flash_read(void)
{
#if FLASH_STORAGE_AVAILABLE
    ppg_paused_for_flash_read = false;
    LOG_INF("PPG processing RESUMED after flash read");
#endif
}

/******************************************************************************
 *                              THREAD DEFINITIONS                            *
 ******************************************************************************/

/* Define PPG management thread with HRM processing: 8KB stack, priority -1 */
K_THREAD_DEFINE(ppg_thread, 8196, ppg_thread_main, NULL, NULL, NULL, 1, K_ESSENTIAL, 0);

/* Define accelerometer stub thread: 2KB stack, priority -2 */
K_THREAD_DEFINE(acc_thread, 2048, acc_sampling_thread, NULL, NULL, NULL, 2, 0, 0);
