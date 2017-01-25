/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      main.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"

/* Add by Caidy --------------------------------------------------------------*/
#include "common.h"
#define q30						1073741824.0f

#ifdef DEBUG_PRINT
	#define MPL_LOGI			printf
	#define MPL_LOGE			printf
#else
  #define MPL_LOGI  		do {} while (0)
  #define MPL_LOGE  		do {} while (0)
#endif

/* Private typedef -----------------------------------------------------------*/
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

/* Add by Caidy - Simplify Code ----------------------------------------------*/
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

/** Converts an orientation matrix made up of 0,+1,and -1 to a scalar representation.
* @param[in] mtx Orientation matrix to convert to a scalar.
* @return Description of orientation matrix. The lowest 2 bits (0 and 1) represent the column the one is on for the
* first row, with the bit number 2 being the sign. The next 2 bits (3 and 4) represent
* the column the one is on for the second row with bit number 5 being the sign.
* The next 2 bits (6 and 7) represent the column the one is on for the third row with
* bit number 8 being the sign. In binary the identity matrix would therefor be:
* 010_001_000 or 0x88 in hex.
*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}
/* Caidy End - Simplify Code -------------------------------------------------*/

#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("DMP: <Info> Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("DMP: <Info> Accuracy= %d\r\r\n", accuracy);

}
#endif

/* Handle sensor on/off combinations. */
//static void setup_gyro(void)
//{
//    unsigned char mask = 0, lp_accel_was_on = 0;
//    if (hal.sensors & ACCEL_ON)
//        mask |= INV_XYZ_ACCEL;
//    if (hal.sensors & GYRO_ON) {
//        mask |= INV_XYZ_GYRO;
//        lp_accel_was_on |= hal.lp_accel_mode;
//    }
//#ifdef COMPASS_ENABLED
//    if (hal.sensors & COMPASS_ON) {
//        mask |= INV_XYZ_COMPASS;
//        lp_accel_was_on |= hal.lp_accel_mode;
//    }
//#endif
//    /* If you need a power transition, this function should be called with a
//     * mask of the sensors still enabled. The driver turns off any sensors
//     * excluded from this mask.
//     */
//    mpu_set_sensors(mask);
//    mpu_configure_fifo(mask);
//    if (lp_accel_was_on) {
//        //unsigned short rate;
//        hal.lp_accel_mode = 0;
//        /* Switching out of LP accel, notify MPL of new accel sampling rate. */
//        //mpu_get_sample_rate(&rate);
//        //inv_set_accel_sample_rate(1000000L / rate);
//    }
//}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
				MPL_LOGI("DMP: <Info> Passed!\r\n");
        MPL_LOGI("DMP: <Info> Accel: %7.4f %7.4f %7.4f\r\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("DMP: <Info> Gyro: %7.4f %7.4f %7.4f\r\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
    }
    else {
				if (!(result & 0x1))
						MPL_LOGE("DMP: <Error> Gyro failed.\r\n");
				if (!(result & 0x2))
						MPL_LOGE("DMP: <Error> Accel failed.\r\n");
				if (!(result & 0x4))
						MPL_LOGE("DMP: <Error> Compass failed.\r\n");
    }
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
/*******************************************************************************/

void DMP_Init(void)
{
	  int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
#endif
 
		result = mpu_init(&int_param);
		if (result) {
				MPL_LOGE("DMP: <Error> Could not initialize Gyro.\r\n");
		}

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif

    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    result = dmp_load_motion_driver_firmware();
		if (result) {
				MPL_LOGE("DMP: <Error> Could not load Firmware.\r\n");
		}
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
		
		run_self_test();
}

void DMP_GetData(s16 *ACCEL, s16 *GYRO, s16 *QUAT)
{
		if (!hal.sensors || !hal.new_gyro) {
				return;
		}

		if (hal.new_gyro && hal.dmp_on) {
				short gyro[3], accel_short[3], sensors;
				long quat[4];
				unsigned char more;
				unsigned long sensor_timestamp;
				
				/* This function gets new data from the FIFO when the DMP is in
				 * use. The FIFO can contain any combination of gyro, accel,
				 * quaternion, and gesture data. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
				 * the FIFO isn't being filled with accel data.
				 * The driver parses the gesture data to determine if a gesture
				 * event has occurred; on an event, the application will be notified
				 * via a callback (assuming that a callback function was properly
				 * registered). The more parameter is non-zero if there are
				 * leftover packets in the FIFO.
				 */
				dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
				if (!more)
						hal.new_gyro = 0;
				
				if (sensors & INV_XYZ_ACCEL) {
						ACCEL[0] = (s16)((float)accel_short[0]/32768*2*100);		//X, 放大100倍
						ACCEL[1] = (s16)((float)accel_short[1]/32768*2*100);		//Y, 放大100倍
						ACCEL[2] = (s16)((float)accel_short[2]/32768*2*100);		//Z, 放大100倍
				}
				
				if (sensors & INV_XYZ_GYRO) {
						GYRO[0] = (s16)((float)gyro[0]/32768*2000*100);		//X, 放大100倍
						GYRO[1] = (s16)((float)gyro[1]/32768*2000*100);		//Y, 放大100倍
						GYRO[2] = (s16)((float)gyro[2]/32768*2000*100);		//Z, 放大100倍
				}
				
				if (sensors & INV_WXYZ_QUAT) {
						float w=0.0f, x=0.0f, y=0.0f, z=0.0f;
			
						w = quat[0]/q30;
						x = quat[1]/q30;
						y = quat[2]/q30;
						z = quat[3]/q30;

						QUAT[0] = (s16)(atan2(2*y*z + 2*w*x, -2*x*x - 2*y*y + 1)*57.3*100);		// Pitch, 放大100倍
						QUAT[1] = (s16)(asin (2*w*y - 2*x*z)*57.3*100);		// Roll, 放大100倍
						QUAT[2] = (s16)(atan2(2*x*y + 2*w*z, w*w + x*x - y*y - z*z)*57.3*100);		//, 放大100倍
				}
		}
}
