#if (HAS_MPU)

#include "globals.h"
#include "mpuread.h"

// Local logging tag
static const char TAG[] = __FILE__;

MPU6050 mpu;

TaskHandle_t MPUTask;

bool dmp_ready = false; // set true if DMP init was successful
volatile bool mpu_interrupt = false; // indicates whether MPU interrupt pin has gone high
uint8_t mpu_int_status;               // holds actual interrupt status byte from MPU
uint16_t packet_size = 0;
uint8_t dev_status = 0;  
uint16_t fifo_count;                  // count of all bytes currently in FIFO
uint8_t fifo_buffer[64];              // FIFO storage buffer
const uint8_t NUM_SAMPLES = 10;
uint8_t cur_sample = 0;

struct mpu_samples_t
{
  uint32_t ts;
  Quaternion q;
  int16_t gX;
  int16_t gY;
  int16_t gZ;
  int16_t aX;
  int16_t aY;
  int16_t aZ;
} mpu_samples[NUM_SAMPLES];

void mpu_dmpDataReady()
{
  mpu_interrupt = true;
}

// initialize and configure MPU
int mpu_init(void) {
  Wire.begin();
  Wire.setClock(400000);

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
    ESP_LOGI(TAG, "Enabling interrupt detection (Arduino external interrupt %d )...", MPU_INTERRUPT_PIN);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), mpu_dmpDataReady, RISING);
    mpu_int_status = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    ESP_LOGI(TAG, "DMP ready! Waiting for first interrupt...");
    dmp_ready = true;

    // get expected DMP packet size for later comparison
    packet_size = mpu.dmpGetFIFOPacketSize();
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
  return !dev_status;
}

// MPU serial feed FreeRTos Task
void mpu_loop(void *pvParameters) {
  configASSERT(((uint32_t)pvParameters) == 1); // FreeRTOS check

  while(true) {
    fifo_count = mpu.getFIFOCount();

    ESP_LOGD(TAG, "packet_size: %d | fifo_count: %d", packet_size, fifo_count);
    
    while (fifo_count >= packet_size)
    {
      mpu.getFIFOBytes(fifo_buffer, packet_size);
      fifo_count -= packet_size;
      mpu_samples[cur_sample].ts = now();
      mpu.dmpGetQuaternion(&mpu_samples[cur_sample].q, fifo_buffer);
      mpu_samples[cur_sample].gX = (fifo_buffer[16] << 8) | fifo_buffer[17];
      mpu_samples[cur_sample].gY = (fifo_buffer[20] << 8) | fifo_buffer[21];
      mpu_samples[cur_sample].gZ = (fifo_buffer[24] << 8) | fifo_buffer[25];
      mpu_samples[cur_sample].aX = (fifo_buffer[28] << 8) | fifo_buffer[29];
      mpu_samples[cur_sample].aY = (fifo_buffer[32] << 8) | fifo_buffer[33];
      mpu_samples[cur_sample].aZ = (fifo_buffer[36] << 8) | fifo_buffer[37];
      cur_sample++;
      if(cur_sample == NUM_SAMPLES) {
        cur_sample = 0;
      }
    }
    delay(2); // yield to CPU
  }
}

#endif // HAS_MPU