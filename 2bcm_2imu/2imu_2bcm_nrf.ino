#include <Arduino.h>
#include <SPI.h>
#include <mic.h>
#include <hal/nrf_pdm.h>


struct IMUPairSample;

/* ========== PDM(BCM) setup：2 buffer + stream out every 1600 sample  ========== */
#define DEBUG 1
#define SAMPLES 1600
#define SERIAL_BAUD 2000000  

//audio frame header
const uint8_t audioHeader[] = {
  0x00, 0x11, 0x22, 0x33, (SAMPLES >> 8) & 0xFF, SAMPLES & 0xFF
};

mic_config_t mic_config{
  .channel_cnt   = 2,
  .sampling_rate = 41667,    // samplerate
  .buf_size      = 1600,
  .debug_pin     = LED_BUILTIN
};

NRF52840_ADC_Class Mic(&mic_config);

static int16_t bufA[SAMPLES];
static int16_t bufB[SAMPLES];
volatile uint8_t  active_buf = 0;   // 0->A, 1->B
volatile uint16_t fill_idx   = 0;
volatile bool A_ready = false;
volatile bool B_ready = false;

static void send_audio_frame(const int16_t *data, size_t n) {
  Serial.write(audioHeader, 6);
  Serial.write((const uint8_t*)data, n * sizeof(int16_t));
}

/* ========== IMU(dual MPU-9250) setup：100Hz sample_rate，stream out every 100 groups ========== */
static const int CS1 = 5;
static const int CS2 = 4;
static SPISettings mpuSPI(1000000, MSBFIRST, SPI_MODE3);

// 100 Hz -> 10ms
static const uint32_t IMU_PERIOD_US = 10000UL;

// MPU-9250 / AK8963 
#define MPU_WHO_AM_I         0x75
#define MPU_PWR_MGMT_1       0x6B
#define MPU_SMPLRT_DIV       0x19
#define MPU_CONFIG           0x1A
#define MPU_GYRO_CONFIG      0x1B
#define MPU_ACCEL_CONFIG     0x1C
#define MPU_ACCEL_CONFIG2    0x1D
#define MPU_INT_PIN_CFG      0x37
#define MPU_USER_CTRL        0x6A
#define MPU_I2C_MST_CTRL     0x24
#define MPU_I2C_SLV0_ADDR    0x25
#define MPU_I2C_SLV0_REG     0x26
#define MPU_I2C_SLV0_CTRL    0x27
#define MPU_EXT_SENS_DATA_00 0x49
#define MPU_ACCEL_XOUT_H     0x3B
#define MPU_I2C_SLV0_DO      0x63

#define AK8963_I2C_ADDR   0x0C
#define AK8963_WHO_AM_I   0x00
#define AK8963_ST1        0x02
#define AK8963_HXL        0x03
#define AK8963_ST2        0x09
#define AK8963_CNTL1      0x0A
#define AK8963_CNTL2      0x0B
#define AK8963_MODE_16BIT_100HZ 0x16
#define AK8963_RESET            0x01

struct IMUState { int cs; };
IMUState imu1{CS1}, imu2{CS2};

inline int otherCS(int cs){ return (cs==CS1)?CS2:CS1; }
inline void csSelectSafe(int cs){ digitalWrite(otherCS(cs), HIGH); digitalWrite(cs, LOW); }
inline void csDeselectSafe(int cs){ digitalWrite(cs, HIGH); }

uint8_t mpuReadReg(int cs, uint8_t reg){
  uint8_t v;
  SPI.beginTransaction(mpuSPI);
  csSelectSafe(cs);
  SPI.transfer(reg | 0x80);
  v = SPI.transfer(0x00);
  csDeselectSafe(cs);
  SPI.endTransaction();
  return v;
}
void mpuWriteReg(int cs, uint8_t reg, uint8_t data){
  SPI.beginTransaction(mpuSPI);
  csSelectSafe(cs);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(data);
  csDeselectSafe(cs);
  SPI.endTransaction();
}
void mpuReadBytes(int cs, uint8_t reg, uint8_t *buf, size_t len){
  SPI.beginTransaction(mpuSPI);
  csSelectSafe(cs);
  SPI.transfer(reg | 0x80);
  for(size_t i=0;i<len;++i) buf[i]=SPI.transfer(0x00);
  csDeselectSafe(cs);
  SPI.endTransaction();
}

// AK8963:（ADDR->REG->DO->CTRL）
static void ak8963Write1(int cs, uint8_t reg, uint8_t data){
  mpuWriteReg(cs, MPU_I2C_SLV0_ADDR, AK8963_I2C_ADDR); // write
  mpuWriteReg(cs, MPU_I2C_SLV0_REG,  reg);
  mpuWriteReg(cs, MPU_I2C_SLV0_DO,   data);
  mpuWriteReg(cs, MPU_I2C_SLV0_CTRL, 0x81);            // EN|LEN=1
  delay(10);
}

// through MPU I2C nrf read data from AK8963 to EXT_SENS_DATA_00
static void ak8963ReadN(int cs, uint8_t startReg, uint8_t *buf, uint8_t len){
  mpuWriteReg(cs, MPU_I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR); // read
  mpuWriteReg(cs, MPU_I2C_SLV0_REG,  startReg);
  mpuWriteReg(cs, MPU_I2C_SLV0_CTRL, 0x80 | (len & 0x0F));    // EN|LEN
  // delay(2); 
  mpuReadBytes(cs, MPU_EXT_SENS_DATA_00, buf, len);
}

static bool ak8963Init(int cs){
  ak8963Write1(cs, AK8963_CNTL2, AK8963_RESET);
  delay(50);
  uint8_t who;
  ak8963ReadN(cs, AK8963_WHO_AM_I, &who, 1);
  if (who != 0x48){
    Serial.print(F("[CS=")); Serial.print(cs);
    Serial.print(F("] AK8963 WHO_AM_I=0x")); Serial.println(who, HEX);
  }
  // continously reading：100Hz、16-bit
  ak8963Write1(cs, AK8963_CNTL1, AK8963_MODE_16BIT_100HZ);
  delay(20);
  return true;
}

static bool readMagRaw_poll(int cs, int16_t &mx,int16_t &my,int16_t &mz){
  uint8_t raw[6];
  ak8963ReadN(cs, AK8963_HXL, raw, 6);
  uint8_t st2 = 0;
  ak8963ReadN(cs, AK8963_ST2, &st2, 1);
  // AK8963 
  mx = (int16_t)((raw[1] << 8) | raw[0]);
  my = (int16_t)((raw[3] << 8) | raw[2]);
  mz = (int16_t)((raw[5] << 8) | raw[4]);
  return true;
}

// MPU initial：set ODR 100Hz（1kHz/(1+9)）
static bool mpuInit(int cs){
  pinMode(cs, OUTPUT);
  csDeselectSafe(cs);

  mpuWriteReg(cs, MPU_PWR_MGMT_1, 0x00); delay(50);
  mpuWriteReg(cs, MPU_INT_PIN_CFG, 0x00); 

  mpuWriteReg(cs, MPU_CONFIG,        0x03); // DLPF ~44Hz
  mpuWriteReg(cs, MPU_SMPLRT_DIV,    9);    // 100 Hz
  mpuWriteReg(cs, MPU_GYRO_CONFIG,   0x00); // ±250 dps
  mpuWriteReg(cs, MPU_ACCEL_CONFIG,  0x00); // ±2g
  mpuWriteReg(cs, MPU_ACCEL_CONFIG2, 0x03);

  // I2C Master initial 
  mpuWriteReg(cs, MPU_USER_CTRL, 0x00);  delay(1);
  mpuWriteReg(cs, MPU_USER_CTRL, 0x20);  delay(1); // I2C_MST_EN
  mpuWriteReg(cs, MPU_USER_CTRL, 0x22);  delay(2); // I2C_MST_EN|RESET
  mpuWriteReg(cs, MPU_I2C_MST_CTRL, 0x0D);

  ak8963Init(cs);
  return true;
}

static void readAccelGyroRaw(int cs,
                             int16_t &ax,int16_t &ay,int16_t &az,
                             int16_t &gx,int16_t &gy,int16_t &gz){
  uint8_t b[14];
  mpuReadBytes(cs, MPU_ACCEL_XOUT_H, b, 14);
  ax=(int16_t)((b[0]<<8)|b[1]);
  ay=(int16_t)((b[2]<<8)|b[3]);
  az=(int16_t)((b[4]<<8)|b[5]);
  gx=(int16_t)((b[8]<<8)|b[9]);
  gy=(int16_t)((b[10]<<8)|b[11]);
  gz=(int16_t)((b[12]<<8)|b[13]);
}

/* ===== IMU double buffers ===== */
struct IMUPairSample { int16_t a1[3], g1[3], m1[3]; int16_t a2[3], g2[3], m2[3]; };
#define IMU_BATCH 100

static IMUPairSample imuBufA[IMU_BATCH];
static IMUPairSample imuBufB[IMU_BATCH];

volatile uint8_t  imu_active = 0;   // 0->A, 1->B
volatile uint16_t imu_fill   = 0;   
volatile bool imuA_ready = false;
volatile bool imuB_ready = false;

static uint32_t imu_last_t_us = 0;
static uint32_t imu_accum_us  = 0;

// stream out IMU data
static void send_imu_block(const IMUPairSample* buf, uint16_t count){
  const uint8_t tag[4] = {'I','M','U','2'};
  uint8_t hdr[6] = {tag[0],tag[1],tag[2],tag[3], (uint8_t)(count>>8), (uint8_t)(count&0xFF)};
  Serial.write(hdr, 6);
  for(uint16_t i=0;i<count;++i){
    const IMUPairSample &s = buf[i];
    Serial.write((const uint8_t*)s.a1, 3*sizeof(int16_t));
    Serial.write((const uint8_t*)s.g1, 3*sizeof(int16_t));
    Serial.write((const uint8_t*)s.m1, 3*sizeof(int16_t));
    Serial.write((const uint8_t*)s.a2, 3*sizeof(int16_t));
    Serial.write((const uint8_t*)s.g2, 3*sizeof(int16_t));
    Serial.write((const uint8_t*)s.m2, 3*sizeof(int16_t));
  }
}

// sampling  IMU 
static inline void imu_sample_once(){
  IMUPairSample s;

  // IMU1
  readAccelGyroRaw(CS1, s.a1[0],s.a1[1],s.a1[2], s.g1[0],s.g1[1],s.g1[2]);
  if (!readMagRaw_poll(CS1, s.m1[0],s.m1[1],s.m1[2])) { s.m1[0]=s.m1[1]=s.m1[2]=0; }

  // IMU2
  readAccelGyroRaw(CS2, s.a2[0],s.a2[1],s.a2[2], s.g2[0],s.g2[1],s.g2[2]);
  if (!readMagRaw_poll(CS2, s.m2[0],s.m2[1],s.m2[2])) { s.m2[0]=s.m2[1]=s.m2[2]=0; }

  if (imu_active == 0) imuBufA[imu_fill] = s;
  else                 imuBufB[imu_fill] = s;

  imu_fill++;
  if (imu_fill >= IMU_BATCH){
    if (imu_active == 0){ imuA_ready = true; imu_active = 1; }
    else                { imuB_ready = true; imu_active = 0; }
    imu_fill = 0;
  }
}


static void maybeSampleIMUs(){
  uint32_t now_us = micros();
  uint32_t dt     = (uint32_t)(now_us - imu_last_t_us);  
  imu_last_t_us   = now_us;

  imu_accum_us += dt;

  uint32_t due = imu_accum_us / IMU_PERIOD_US;   
  if (due == 0) return;
  imu_accum_us -= due * IMU_PERIOD_US;

  const uint32_t kMaxCatchUpPerLoop = 5;
  if (due > kMaxCatchUpPerLoop) due = kMaxCatchUpPerLoop;

  while (due--) {
    imu_sample_once();
  }
}

/* audio callback*/
static void audio_rec_callback(uint16_t *buf, uint32_t buf_len){
  while (buf_len--){
    if (active_buf == 0){
      bufA[fill_idx++] = *buf++;
      if (fill_idx >= SAMPLES){
        A_ready   = true;
        active_buf = 1;
        fill_idx   = 0;
      }
    }else{
      bufB[fill_idx++] = *buf++;
      if (fill_idx >= SAMPLES){
        B_ready   = true;
        active_buf = 0;
        fill_idx   = 0;
      }
    }
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  while(!Serial){ delay(10); }

  // PDM
  Mic.set_callback(audio_rec_callback);
  if (!Mic.begin()){
    Serial.println("Mic initialization failed");
    while(1);
  }
  nrf_pdm_gain_set(100, 100);
  Serial.println("Mic initialization done.");

  // IMU
  pinMode(CS1, OUTPUT); pinMode(CS2, OUTPUT);
  csDeselectSafe(CS1);  csDeselectSafe(CS2);
  SPI.begin();
  mpuInit(imu1.cs);
  mpuInit(imu2.cs);

  // initial counter
  imu_last_t_us = micros();
  imu_accum_us  = 0;
}

void loop(){
  // sample 100Hz IMU
  maybeSampleIMUs();

  // bcm_audio
  if (A_ready){ send_audio_frame(bufA, SAMPLES); A_ready = false; }
  if (B_ready){ send_audio_frame(bufB, SAMPLES); B_ready = false; }

  // IMU
  if (imuA_ready){ send_imu_block(imuBufA, IMU_BATCH); imuA_ready = false; }
  if (imuB_ready){ send_imu_block(imuBufB, IMU_BATCH); imuB_ready = false; }
}
