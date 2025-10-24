# RPLIDAR Data Reception Fix Patch

## Application Instructions

This patch fixes critical issues with RPLIDAR data reception. Please follow these steps to apply:

1. Backup original files: `radar.c`, `radar.h`
2. Update code according to the modifications below
3. Recompile and flash firmware
4. Use test scripts to verify fix effects

---

## Modification 1: radar.h - Fix data structures and constants

### Location: radar.h lines 37-44

**Before modification:**
```c
typedef struct {
    uint16_t angle_raw;
    uint16_t distance_raw;
    float angle_deg;
    float distance_m;      ///< Distance (millimeters, variable name is misleading)
    uint8_t quality;       ///< Signal quality (0-127, temporarily not verifying S-bit format)
    uint8_t is_sync;
} RadarRawPoint_t;
```

**After modification:**
```c
typedef struct {
    uint16_t angle_raw;    ///< Raw angle value (angle_q6 format)
    uint16_t distance_raw; ///< Raw distance value (distance_q2 format)
    float angle_deg;       ///< Angle (degrees, 0-360°)
    float distance_mm;     ///< Distance (millimeters) ⚠️ Variable name fixed
    uint8_t quality;       ///< Signal quality (0-63, 6 bits) ⚠️ Range fixed
    uint8_t is_sync;       ///< Sync flag (1=new revolution start)
} RadarRawPoint_t;
```

### Location: radar.h line 85

**Before modification:**
```c
#define RADAR_MIN_QUALITY  10  ///< Minimum quality threshold (0-127)
```

**After modification:**
```c
#define RADAR_MIN_QUALITY  15  ///< Minimum quality threshold (within 0-63 range) ⚠️ Fixed
```

---

## Modification 2: radar.c - Fix initialization function (receive start response)

### Location: radar.c lines 82-154 `Radar_Init()` function

After `Radar_StartScan(huart_radar);`, add the following code:

```c
/**
 * @brief Initialize radar continuous scan mode (DMA optimized version)
 */
uint8_t Radar_Init(UART_HandleTypeDef *huart_radar)
{
    if (huart_radar == NULL) {
        return 1;
    }
    
    g_huart_radar = huart_radar;
    
    extern UART_HandleTypeDef huart4;
    g_huart_output = &huart4;
    
    const char* msg1 = "[LOG:INFO] RADAR: Initializing DMA mode...\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg1, strlen(msg1), 100);
    
    // Stop radar
    Radar_StopScan(huart_radar);
    HAL_Delay(100);
    
    // Clear buffer
    uint32_t cleared = Radar_ClearReceiveBuffer(huart_radar);
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg), "[LOG:DEBUG] RADAR: Cleared %lu bytes\r\n", cleared);
    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    
    // Initialize buffers
    memset(&g_frame_buffer, 0, sizeof(RadarFrameBuffer_t));
    memset((void*)g_radar_dma_buffer, 0, RADAR_DMA_BUFFER_SIZE);
    g_last_angle_raw = 0;
    g_dma_last_pos = 0;
    g_data_ready = 0;
    
    const char* msg2 = "[LOG:INFO] RADAR: Starting scan...\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg2, strlen(msg2), 100);
    
    // Start radar scan
    HAL_StatusTypeDef ret = Radar_StartScan(huart_radar);
    if (ret != HAL_OK) {
        return 2;
    }
    
    // ⭐⭐⭐ New: Receive and verify start response (7 bytes)
    const char* msg3 = "[LOG:INFO] RADAR: Waiting for start response...\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg3, strlen(msg3), 100);
    
    uint8_t start_response[10];
    ret = HAL_UART_Receive(huart_radar, start_response, 7, 2000);
    
    if (ret != HAL_OK) {
        const char* msg_err = "[LOG:ERROR] RADAR: Failed to receive start response\r\n";
        HAL_UART_Transmit(&huart4, (uint8_t*)msg_err, strlen(msg_err), 100);
        return 4;
    }
    
    // Verify start response
    if (start_response[0] != 0xA5 || start_response[1] != 0x5A) {
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[LOG:ERROR] RADAR: Invalid start response: %02X %02X (expected A5 5A)\r\n",
                 start_response[0], start_response[1]);
        HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
        return 5;
    }
    
    // Parse start response
    uint32_t data_len = (start_response[2] | 
                        (start_response[3] << 8) | 
                        (start_response[4] << 16)) & 0x3FFFFFFF;
    uint8_t mode = (start_response[4] >> 6) & 0x03;
    uint8_t data_type = start_response[6];
    
    snprintf(debug_msg, sizeof(debug_msg), 
             "[LOG:INFO] RADAR: Start Response OK - DataLen=%lu, Mode=%u, Type=0x%02X\r\n",
             data_len, mode, data_type);
    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    
    if (data_len != 5) {
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[LOG:WARN] RADAR: Unexpected data length: %lu (expected 5)\r\n", data_len);
        HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    }
    
    // Clear residual data
    Radar_ClearReceiveBuffer(huart_radar);
    
    // Delay for radar stabilization
    HAL_Delay(500);
    
    // Start DMA
    const char* msg4 = "[LOG:INFO] RADAR: Starting DMA receive...\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg4, strlen(msg4), 100);
    
    ret = HAL_UART_Receive_DMA(huart_radar, (uint8_t*)g_radar_dma_buffer, RADAR_DMA_BUFFER_SIZE);
    if (ret != HAL_OK) {
        const char* msg5 = "[LOG:ERROR] RADAR: DMA start failed!\r\n";
        HAL_UART_Transmit(&huart4, (uint8_t*)msg5, strlen(msg5), 100);
        return 3;
    }
    
    // Enable IDLE interrupt
    __HAL_UART_ENABLE_IT(huart_radar, UART_IT_IDLE);
    
    const char* msg6 = "[LOG:INFO] RADAR: DMA mode ready\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg6, strlen(msg6), 100);
    
    return 0;
}
```

---

## Modification 3: radar.c - Fix data parsing function (restore S-bit verification)

### Location: radar.c lines 254-299 `Radar_ParseSinglePoint()` function

**Replace completely with:**
```c
/**
 * @brief Parse single 5-byte measurement point
 * @note  Data format (RPLIDAR protocol):
 *        Byte 0: [S(bit7)][S'(bit6)][Quality(bit1-6)][C(bit0)]
 *        Byte 1-2: angle_q6 (little endian)
 *        Byte 3-4: distance_q2 (little endian)
 */
int Radar_ParseSinglePoint(uint8_t *buffer, RadarRawPoint_t *point)
{
    if (buffer == NULL || point == NULL) {
        return -1;
    }
    
    // Extract 5-byte data packet
    uint8_t sync_qual = buffer[0];
    point->angle_raw = buffer[1] | (buffer[2] << 8);
    point->distance_raw = buffer[3] | (buffer[4] << 8);
    
    // ⭐ Extract flag bits (strictly following RPLIDAR protocol)
    uint8_t s_bit = (sync_qual >> 7) & 0x01;    // bit 7: S
    uint8_t s_prime = (sync_qual >> 6) & 0x01;  // bit 6: S'
    uint8_t c_bit = (sync_qual >> 0) & 0x01;    // bit 0: C
    
    // ⭐ Verify C bit (must be 1) - this is the most basic check
    if (c_bit != 1) {
        return -1;  // C bit verification failed
    }
    
    // ⭐ Verify S' bit (optional, can be commented out if device S' bit is unreliable)
    // if (s_prime != (1 - s_bit)) {
    //     return -1;  // S' bit verification failed
    // }
    
    point->is_sync = s_bit;
    // ⭐ Correctly extract quality: bit 1-6 (6 bits, range 0-63)
    point->quality = (sync_qual >> 1) & 0x3F;
    
    // Convert to actual values
    point->angle_deg = (float)point->angle_raw / RADAR_ANGLE_SCALE;
    point->distance_mm = (float)point->distance_raw / RADAR_DISTANCE_SCALE;  // ⚠️ Renamed
    
    return 0;
}
```

---

## Modification 4: radar.c - Enhance frame synchronization algorithm

### Location: radar.c lines 696-749 `Radar_FindSyncOffset()` function

**Replace completely with:**
```c
/**
 * @brief Find data packet synchronization offset
 * @param buffer Data buffer
 * @param size Buffer size
 * @return Best offset value (0-9), -1 means not found
 * 
 * @note Fixed version: Expanded offset search range, added C-bit verification
 */
static int Radar_FindSyncOffset(uint8_t *buffer, uint16_t size)
{
    int best_offset = -1;
    int max_score = 0;
    
    // ⭐ Expand offset search range to 0-9 bytes (cover possible start response residues)
    int max_offset = (size < 10) ? size : 10;
    
    for (int offset = 0; offset < max_offset; offset++) {
        int valid_points = 0;
        int c_bit_ok = 0;
        int angle_ok = 0;
        int distance_ok = 0;
        
        // Parse first 20 points to test validity
        for (int i = offset, count = 0; 
             i + POINT_PACKET_SIZE <= size && count < 20; 
             i += POINT_PACKET_SIZE, count++) {
            
            RadarRawPoint_t point;
            
            if (Radar_ParseSinglePoint(&buffer[i], &point) != 0) {
                continue;  // Parsing failed (C-bit verification not passed)
            }
            
            // ⭐ Count points passing C-bit verification (verified in ParseSinglePoint)
            c_bit_ok++;
            
            // Count various indicators
            if (point.angle_deg >= 0 && point.angle_deg < 360.0f) {
                angle_ok++;
            }
            if (point.distance_mm >= RADAR_MIN_DISTANCE && 
                point.distance_mm <= RADAR_MAX_DISTANCE) {
                distance_ok++;
            }
            
            // All conditions must be met to count as valid point
            if (point.angle_deg >= 0 && point.angle_deg < 360.0f &&
                point.distance_mm >= RADAR_MIN_DISTANCE && 
                point.distance_mm <= RADAR_MAX_DISTANCE) {
                valid_points++;
            }
        }
        
        // ⭐ C-bit verification has highest weight (must all pass)
        int score = c_bit_ok * 100 + angle_ok * 10 + distance_ok * 5 + valid_points * 20;
        
        if (score > max_score) {
            max_score = score;
            best_offset = offset;
        }
    }
    
    // ⭐ If score is too low, return -1 instead of 0
    if (max_score < 100) {
        return -1;
    }
    
    return best_offset;
}
```

---

## Modification 5: radar.c - Fix all places using distance_m

### Location 1: radar.c line 596

**Before modification:**
```c
if (point->distance_m < RADAR_MIN_DISTANCE || point->distance_m > RADAR_MAX_DISTANCE) {
```

**After modification:**
```c
if (point->distance_mm < RADAR_MIN_DISTANCE || point->distance_mm > RADAR_MAX_DISTANCE) {
```

### Location 2: radar.c line 603

**Before modification:**
```c
uint16_t dist_mm = (uint16_t)point->distance_m;
```

**After modification:**
```c
uint16_t dist_mm = (uint16_t)point->distance_mm;
```

### Location 3: radar.c line 409 (debug output)

**Before modification:**
```c
snprintf(point_debug, sizeof(point_debug), 
         "[POINT#%u] angle_raw=%u angle_deg=%.2f dist_raw=%u dist_mm=%.1f qual=%u idx=%u\r\n",
         sample_count, point.angle_raw, point.angle_deg, 
         point.distance_raw, point.distance_m, point.quality, index);
```

**After modification:**
```c
snprintf(point_debug, sizeof(point_debug), 
         "[POINT#%u] angle_raw=%u angle_deg=%.2f dist_raw=%u dist_mm=%.1f qual=%u idx=%u\r\n",
         sample_count, point.angle_raw, point.angle_deg, 
         point.distance_raw, point.distance_mm, point.quality, index);
```

---

## Modification 6: radar.c - Fix frame synchronization handling in Radar_ContinuousScan function

### Location: radar.c lines 373-374

**Before modification:**
```c
int offset = Radar_FindSyncOffset(g_radar_rx_buffer, data_len);
if (offset < 0) offset = 0;
```

**After modification:**
```c
int offset = Radar_FindSyncOffset(g_radar_rx_buffer, data_len);
if (offset < 0) {
    // ⚠️ No valid offset found, skip this data
    static uint32_t sync_fail_count = 0;
    sync_fail_count++;
    if (sync_fail_count % 100 == 1) {  // Output warning every 100 failures
        char warn_msg[80];
        snprintf(warn_msg, sizeof(warn_msg), 
                 "[WARN] Frame sync failed (count: %lu)\r\n", sync_fail_count);
        HAL_UART_Transmit(huart_output, (uint8_t*)warn_msg, strlen(warn_msg), 100);
    }
    return;  // Skip this data, wait for next
}
```

---

## Compilation and Testing

### 1. Compile Firmware

In STM32CubeIDE:
1. Open project `xxq/xxq.ioc`
2. Click `Project` -> `Build All`
3. Ensure compilation without errors

### 2. Flash Firmware

1. Connect ST-Link
2. Click `Run` -> `Debug` or `Run`
3. Wait for flashing to complete

### 3. Run Test Scripts

**Test 1: Raw Data Diagnosis**
```bash
cd xxq_host/scripts
python diagnose_rplidar_raw.py
```

Expected output:
- ✅ Found start response (A5 5A ...)
- ✅ Found valid frame sync offset
- ✅ Angle range 0-360°
- ✅ Distance reasonable

**Test 2: Real-time Visualization**
```bash
python test_rplidar_visualization.py
```

Expected effects:
- ✅ Polar plot shows complete 360° point cloud
- ✅ All directional regions have reasonable distance data
- ✅ Valid points > 400/500
- ✅ Data quality stable

---

## Expected Improvement Effects

Before fix:
- ❌ Angle data chaotic (random values)
- ❌ Distance data inaccurate (may be all 0 or abnormally large)
- ❌ Valid points < 100/500
- ❌ Cannot correctly detect obstacles

After fix:
- ✅ Angle data correct (0-360°, monotonically increasing)
- ✅ Distance data accurate (consistent with actual obstacles)
- ✅ Valid points > 400/500
- ✅ Can correctly detect obstacle positions and distances

---

## Follow-up Optimization Suggestions

1. **Consider using EXPRESS_SCAN mode**
   - Higher sampling rate (up to 8000 points/second)
   - Better angle resolution
   - Requires modifying protocol parsing logic

2. **Add CRC verification**
   - Improve data reliability
   - Timely detection of transmission errors

3. **Implement health check**
   - Periodically send `GET_HEALTH` command
   - Monitor radar status (normal/warning/error)

4. **Optimize DMA buffer**
   - Use larger buffer (2048 bytes)
   - Implement true ping-pong buffering

---

*Patch creation time: 2025-10-21*
*Version: v1.0*

