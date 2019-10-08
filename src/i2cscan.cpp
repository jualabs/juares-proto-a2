// Basic config
#include "globals.h"
#include "i2cscan.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Local logging tag
static const char TAG[] = __FILE__;

#define SSD1306_PRIMARY_ADDRESS (0x3D)
#define SSD1306_SECONDARY_ADDRESS (0x3C)
#define BME_PRIMARY_ADDRESS (0x77)
#define BME_SECONDARY_ADDRESS (0x76)
#define AXP192_PRIMARY_ADDRESS (0x34)
#define MCP_24AA02E64_PRIMARY_ADDRESS (0x50)
#define QUECTEL_GPS_PRIMARY_ADDRESS (0x10)
#define MPU6050_PRIMARY_ADDRESS MPU6050_ADDRESS_AD0_LOW
#define MPU6050_SECONDARY_ADDRESS MPU6050_ADDRESS_AD0_HIGH

#ifdef HAS_MPU

void MPU6050_init(void) {
  MPU6050 mpu;
  uint8_t dev_status = 0;
  
  ESP_LOGI(TAG, "Initializing MPU-6050");
  mpu.initialize();
  pinMode(MPU_INTERRUPT_PIN, INPUT);
  ESP_LOGI(TAG, "Testing device connections...");
  uint8_t connection_status = mpu.testConnection();
  if (connection_status) {
    ESP_LOGI(TAG, "MPU6050 connection successful");
  }
  else {
    ESP_LOGI(TAG, "MPU6050 connection failed");
  }
  ESP_LOGI(TAG, "Initializing DMP...");
  dev_status = mpu.dmpInitialize();
  // TODO: verify this calibrations parameters
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (dev_status == 0)
  {
    // calibration time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    ESP_LOGI(TAG, F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // ESP_LOGI(TAG, "Enabling interrupt detection (Arduino external interrupt ");
    // ESP_LOGI(TAG, digitalPinToInterrupt(MPU_INTERRUPT_PIN));
    // ESP_LOGI(TAG, ")...");
    // attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    // mpu_int_status = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    ESP_LOGI(TAG, "DMP ready! Waiting for first interrupt...");
    //dmp_ready = true;

    // get expected DMP packet size for later comparison
    // packet_size = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    ESP_LOGI(TAG, "DMP Initialization failed (code %d", dev_status);
    ESP_LOGI(TAG, ")");
  }
    
}

#endif // HAS_MPU

int i2c_scan(void) {
  int i2c_ret, addr;
  int devices = 0;

  ESP_LOGI(TAG, "Starting I2C bus scan...");

  for (addr = 8; addr <= 119; addr++) {

    // scan i2c bus with no more to 100KHz
    // Wire.begin(SDA, SCL, 100000);
    Wire.begin(SDA, SCL, 400000);
    Wire.beginTransmission(addr);
    Wire.write(addr);
    i2c_ret = Wire.endTransmission();

    if (i2c_ret == 0) {
      devices++;

      switch (addr) {

      case SSD1306_PRIMARY_ADDRESS:
      case SSD1306_SECONDARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: SSD1306 Display controller", addr);
        break;

      case BME_PRIMARY_ADDRESS:
      case BME_SECONDARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: Bosch BME MEMS", addr);
        break;

      case AXP192_PRIMARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: AXP192 power management", addr);
#ifdef HAS_PMU
        AXP192_init();
#endif
        break;

      case QUECTEL_GPS_PRIMARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: Quectel GPS", addr);
        break;

      case MCP_24AA02E64_PRIMARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: 24AA02E64 serial EEPROM", addr);
        break;
      
      case MPU6050_PRIMARY_ADDRESS:
      case MPU6050_SECONDARY_ADDRESS:
        ESP_LOGI(TAG, "0x%X: MPU-6050", addr);
#ifdef HAS_MPU
        MPU6050_init();
#endif
        break;

      default:
        ESP_LOGI(TAG, "0x%X: Unknown device", addr);
        break;
      }
    } // switch
  }   // for loop

  ESP_LOGI(TAG, "I2C scan done, %u devices found.", devices);

  return devices;
}

#ifdef HAS_PMU

void AXP192_init(void) {

  AXP20X_Class axp;

  if (axp.begin(Wire, AXP192_PRIMARY_ADDRESS))
    ESP_LOGI(TAG, "AXP192 PMU initialization failed");
  else {

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);
    axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
    //axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);

#ifdef PMU_INT
    pinMode(PMU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PMU_INT),
                    [] {
                      ESP_LOGI(TAG, "Power source changed");
                      /* put your code here */
                    },
                    FALLING);
    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ |
                      AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ,
                  1);
    axp.clearIRQ();
#endif // PMU_INT

    ESP_LOGI(TAG, "AXP192 PMU initialized.");
  }
}
#endif // HAS_PMU