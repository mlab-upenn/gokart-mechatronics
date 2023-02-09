#include "cmsis_os.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "appI2C.h"
#include "appUSART.h"
#include "lsm9ds1_reg.h"
#include "appLSM9DS1.h"
#include <string.h>
#include <stdio.h>
 
#define SENSOR_BUS hi2c1
#define BOOT_TIME  20

typedef struct {
  void   *hbus;
  uint8_t i2c_address;
} sensbus_t;

static sensbus_t mag_bus = {&SENSOR_BUS,
                            LSM9DS1_MAG_I2C_ADD_H,
                           };
static sensbus_t imu_bus = {&SENSOR_BUS,
                            LSM9DS1_IMU_I2C_ADD_H,
                           };

static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];

static lsm9ds1_status_t reg;

static lsm9ds1_id_t whoamI;

static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);

void GetIMUReading(float acc_mg[], float gyr_mdps[], float mag_mg[])
{
    /* Read device status register */
	lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

	if ( reg.status_imu.xlda && reg.status_imu.gda )
	{
	  /* Read imu data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
	                               data_raw_acceleration);
	  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
	                               data_raw_angular_rate);
	  acc_mg[0] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[0]);
	  acc_mg[1] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[1]);
	  acc_mg[2] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[2]);
	  gyr_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[0]);
	  gyr_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[1]);
	  gyr_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[2]);
	}

	if ( reg.status_mag.zyxda )
	{
	  /* Read magnetometer data */
	  memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
	  mag_mg[0] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[0]);
	  mag_mg[1] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[1]);
	  mag_mg[2] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[2]);
	}
}

void IMUInit(void)
{
  static BOOL isInited = FALSE;

  if(isInited == TRUE)
  {
    return;
  }
  isInited = TRUE;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = stm32l4_write_imu;
  dev_ctx_imu.read_reg = stm32l4_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = stm32l4_write_mag;
  dev_ctx_mag.read_reg = stm32l4_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;

  osDelay(BOOT_TIME);

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID)
  {
	while (1)
	{
	  osDelay(1);
	}
  }
  else
  {
    USART1TxStr("whoami passed\r\n");
  }

  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
   lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
   lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
   lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
   lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
   lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ODR_DIV_50);
   lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
   /* Gyroscope filtering chain */
   lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ULTRA_LIGHT);
   lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
   lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                  LSM9DS1_LPF1_HPF_LPF2_OUT);
   /* Set Output Data Rate / Power mode */
   lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
   lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);
}

static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  I2C1Tx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  return 0;
}

static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  I2C1Rx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  return 0;
}

static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  reg |= 0x80;
  I2C1Tx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  return 0;
}

static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  reg |= 0x80;
  I2C1Rx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  return 0;
}

