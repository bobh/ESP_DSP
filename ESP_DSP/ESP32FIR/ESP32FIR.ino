
//  Created by bobolink
//  twitter: @wm6h
//  rev: 20200126

// Copyright 2020 WM6H
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* NOTE:
 *  You need to specify the file name of the wav file on the SD card 
 *  you want to pass through the FIR filter
 *  Also, which 400 Hz filter you want to use-- the bandstop or high pass filter--
 *  by including the desired filer coefficient file.
 *  See STEP 1 and STEP 2 below
 */
 
/*
   Two core ESP-32 Arduino real-time audio demo.
   Tested on Espressif ESP32 Dev board
   Rev. 1 silicon
   Real-Time Samples at 8Ksps for voice audio range (< 4KHz).
   Not for high fidelity music.
   Compatible with Arduino IDE

   Updated to read input test signals from sd card on a
   TTGO T8 Development card
   Added FIR filter
   SD card file is: 8Ksps, mono, WAV (Microsoft) signed 16-bit PCM
   FAT32 formatted SD Card

   http://www.iotsharing.com/2017/07/how-to-use-arduino-esp32-i2s-to-play-wav-music-from-sdcard.html

NOTE:
   SDCARD interface library must be installed manually.

   Installing Additional Arduino Libraries
   https://www.arduino.cc/en/guide/libraries

   Found in top directory
   Sketch->Include Library->Add .ZIP Library    “esp32-micro-sdcard-master.zip”
   #include <mySD.h>

*/

#include <Arduino.h>
#include <driver/adc.h>

//STEP 1
//*******************************************************************
// Initialize "SDcardFileName" with the SD Card file you want to play
// Uncomment one of these:

// a 30 sec 1000 Hz tone
// String SDcardFileName = "sineWAV.wav";

// 2 min of 400 Hz plus 1000Hz
// enable 400Hz filter to eliminate interference
String SDcardFileName = "mix.wav";  

// 400Hz tone for 2 minutes
// if the 400Hz StopBand or HighPass filter is in,
// no sound should be heard 
//String SDcardFileName = "400Hz2m.wav";

//*******************************************************************
/*
    Based on Neil Kolban example for IDF:
    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

//\||||||||||||||||||||||||||
#include <mySD.h>

// minimum library includes for the FIR filter
#include "dsp_err.h"
#include "dsp_err_codes.h"

#include "dsps_fir.h"

//STEP 2
//*******************************************************************
// Include only one of these FIR filters
// Bandstop or HighPass
// see ./design/ directory for filter frequency plots

// 400Hz filters:
//#include "fdaBScoefs.h"//(length 121) 40 dB attenuation BandStop
#include "fdaHPcoefs.h"  //(length 51) 60 dB attenuation HighPass

// filter is switched in/out programmatically over a
// Bluetooth Low Energy (BLE) wireless link. 
// A write of a value to characteristic 
// "beb5483e-36e1-4688-b7f5-ea07361b26a8" toggles the filter in/out
// Use free phone app from LightBlue or nRF Connect (Nordic) to
// write values.

// Connect to BLE peripheral advertising name "DSP_FILTER"

//*******************************************************************


// BL is the coefficient name Matlab gives
#define   NTAPS    BL-1

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//\||||||||||||||||||||||||||

//#include "driver/i2s.h"
//#include "freertos/queue.h"

/* variables hold file, state of process wav file and wav file properties */
File root;
bool wavPlaying = false;
bool LoopBack = true;//start with input copied to output to hear tone
// then switch in the filter over BLE



#define CCCC(c1, c2, c3, c4)    ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

/* these are data structures to process wav file */
typedef enum headerState_e {
  HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA, OUT_OF_DATA
} headerState_t;

typedef struct wavRiff_s {
  uint32_t chunkID;
  uint32_t chunkSize;
  uint32_t format;
} wavRiff_t;

typedef struct wavProperties_s {
  uint32_t chunkID;
  uint32_t chunkSize;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
} wavProperties_t;

headerState_t state = HEADER_RIFF;
wavProperties_t wavProps;
// convert little endian to big endian 4 byte int
uint32_t overall_size  = 0;
unsigned char buffer4[4];


const uint16_t  N = 1024; // should be a power of 2 for FFTs
// Determines how long the Application Processor can run for
// real-time applications
// or how long the Application has to write the samples to
// a ring buffer for non-real time applications.
// Latency, input to output, is a function of N.

//\||||||||||||||||||||||||||
// x[] is input to the filter
float x[N];
// y[] is output of the filter and routed to the DAC
float y[N];

float coeffs[NTAPS];
float history[NTAPS];
int len;
int fir_len;
unsigned int startTEST = 0;
unsigned int endTEST = 0;
float totalTEST;

fir_f32_t fir1;

//\||||||||||||||||||||||||||

void debug(uint8_t *buf, int len)
{
  for (int i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print("\t");
  }
  Serial.println();
}

/* read 4 bytes of data from wav file */
int read4bytes(File file, uint32_t *chunkId) {
  int n = file.read((uint8_t *)chunkId, sizeof(uint32_t));
  return n;
}

int readbyte(File file, uint8_t *chunkId) {
  int n = file.read((uint8_t *)chunkId, sizeof(uint8_t));
  return n;
}

/* these are function to process wav file */
int readRiff(File file, wavRiff_t *wavRiff) {
  int n = file.read((uint8_t *)wavRiff, sizeof(wavRiff_t));
  return n;
}
int readProps(File file, wavProperties_t *wavProps) {
  int n = file.read((uint8_t *)wavProps, sizeof(wavProperties_t));
  return n;
}


int LED_BUILTIN_TTGO = 21;

// we create complex numbers here for convenience
// could be done in frame processing
volatile float realPing[N];
volatile float imagPing[N];
volatile float realPong[N];
volatile float imagPong[N];

// Do frame processing in floats for better dynamic range.
// At last stage, scale floats to range -1.0 to +1.0
// then convert to unsigned char. Mult by 127 and
// adding 128 to map +1.0 -- -1.0 to 0-255 for
// output to 8-bit unsigned DAC

// we create analytic IQ input samples
// but we only output real numbers because, well you know,
// they're real
volatile unsigned char outRealPing[N];
volatile unsigned char outRealPong[N];

float volume;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// this is Core 1's variable.
// Core 0 will operate on the other buffer
volatile boolean pingCore1 = true;

volatile boolean sampling = false;
volatile boolean outputEnable = true;


TaskHandle_t Task1;
SemaphoreHandle_t newFrame;

esp_err_t result;

unsigned char __Temp_wav[] = {
  0x00, 0x00
};


unsigned int sampleCNT = 0;

void IRAM_ATTR onTimer()
{

  portENTER_CRITICAL_ISR(&timerMux);
  sampling = true;
  portEXIT_CRITICAL_ISR(&timerMux);

}

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0)
      {
        LoopBack = LoopBack ? false : true;
      }
    }
};


void setup()
{

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN_TTGO, OUTPUT);//it's green
  digitalWrite(LED_BUILTIN_TTGO, LOW);

  Serial.begin(115200);

  Serial.print("Initializing SD card...");

  //  if (!SD.begin(32, 14, 12, 27)) {
  if (!SD.begin(13, 15, 2, 14)) { // for TTGO T8 hardware
    Serial.println("initialization failed!");
    return;
  }

  Serial.println("initialization done.");
  Serial.print("MOSI = "); Serial.println(MOSI);
  Serial.print("MISO = "); Serial.println(MISO);
  Serial.print("SCK = "); Serial.println(SCK);
  Serial.print("SS = "); Serial.println(SS);


  /* open specified wav file and process it */
  root = SD.open(SDcardFileName.c_str ());
  Serial.println(SDcardFileName.c_str ());

  if (root)
  { // we found the file, is it mono wave format?
    int c = 0;
    int n;
    wavRiff_t wavRiff;
    n = readRiff(root, &wavRiff);
    if (n == sizeof(wavRiff_t)) {
      if (wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff.format == CCCC('W', 'A', 'V', 'E')) {
        Serial.println("HEADER_RIFF");
      }
    }

    n = readProps(root, &wavProps);
    if (n == sizeof(wavProperties_t)) {
      Serial.println("WAVE PROPERTIES");
    }

    uint32_t chunkId, chunkSize;
    n = read4bytes(root, &chunkId);
    if (n == 4) {
      if (chunkId == CCCC('d', 'a', 't', 'a'))
      {
        Serial.println("HEADER_DATA");
      }
    }
    //++++++++++++++++++++++++++++++++++++++++=
    // read the 4 bytes chunk size
    n = read4bytes(root, &chunkSize);
    if (n == 4) {

      Serial.println("chunkSize:");
      Serial.println(chunkSize);
      Serial.println("prepare data");
      wavPlaying = true;
    }
    //++++++++++++++++++++++++++++++++++++++++=
    /* after processing wav file, it is time to process audio data */
  }
  else
  {
    Serial.println("error opening wav file");
    wavPlaying = false;
  }


  //\||||||||||||||||||||||||||
  len = sizeof(x) / sizeof(float);
  fir_len = (sizeof(B) / sizeof(float)) - 1; //order v.s. length
  // initialize the coefficients with the Matlab designed values
  for (int i = 0 ; i < fir_len ; i++)
  {
    coeffs[i] = B[i];
  }

  // initialize the fir filter struct using library function
  dsps_fir_init_f32(&fir1, coeffs, history, fir_len);
  //\||||||||||||||||||||||||||

  Serial.println("initialization done.");
  delay(1000);

  newFrame = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    frameProcessing,
    "FrameProcessing",
    1000,
    NULL,
    1,
    &Task1,
    0);

  delay(500);  // needed to start-up task1
  // from Mr. Spiess' video

  // zero the DAC buffers
  for (int i = 0; i < N; ++i)
  {
    outRealPing[i] = 0;
    outRealPong[i] = 0;

  }

  // this almost matches the output resolution
  // all channels  GPIOs 32-39
  result = adc1_config_width(ADC_WIDTH_9Bit);
  // complete with Apple type error message
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }

  // this might allow 3.3VDC on the mic
  // pin 34
  result = adc1_config_channel_atten( ADC1_CHANNEL_6, ADC_ATTEN_11db);
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }

  //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  BLEDevice::init("DSP_FILTER");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

  Serial.print("Arduino Loop(): Executing on core ");
  Serial.println(xPortGetCoreID());

  //Sart the Sample Clock
  hw_timer_t* timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true); // sampling frequency 8kHz
  timerAlarmEnable(timer);

}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
//                   This Task runs on Core: 1
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

// ||||||||||||||||||
//  Sample Service
// ||||||||||||||||||
void loop()
{
  for (int i = 0; i < N; ++i)
  {
    while (!sampling);

    portENTER_CRITICAL(&timerMux);
    sampling = false;
    portEXIT_CRITICAL(&timerMux);

    if (pingCore1 == true)
    {
      // configured for 9 bits with an 11 dB pad
      // drop one bit to give 8 to match the output
      realPing[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPing[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // GPIO pin 25 of the ESP32.
        // This is the output of DAC1
        //dacWrite(25, outRealPing[i]);
        // DAC1 on the TTGO T8 is 26
        dacWrite(26, outRealPing[i]);
      }
    }
    else
    {
      // drop one bit to give 8 to match the output
      realPong[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPong[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // for VDD_A 3.3V, to 2.59V
        //dacWrite(25, outRealPong[i]);
        // DAC1 on the TTGO T8
        dacWrite(26, outRealPong[i]);
      }
    } // end single sample processing
    //digitalWrite(LED_BUILTIN_TTGO, LOW);
  } // end N samples processing

  // swap working buffer
  pingCore1 = !pingCore1;
  // give the old buffer to frame processing
  xSemaphoreGive(newFrame);
} // end sample service task

// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 1111111111111111111111111111111111111111111111111111111111111
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/



// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
//                    This Task runs on Core: 0
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Frame processing
// Runs every N samples

void frameProcessing( void* parameter )
{
  int n;
  uint8_t data;

  Serial.print("Frame Processing: Executing on core ");
  Serial.println(xPortGetCoreID());

  for (;;) // this is required, normally put in by
  {        // Arduino IDE

    int16_t outVal;
    float   outValf;
    uint16_t sample;

    //startTEST = xthal_get_ccount();
    volume = 2.0;
    outputEnable = true;

    // "Arf! Arf!"
    // pet the watchdog.
    vTaskDelay(10);

    // wait for the ping-pong buffer swap
    // indicating frame processing should begin
    // Core 1 has the timer resolution and is in charge of buffers
    // We are Core 0
    xSemaphoreTake(newFrame, portMAX_DELAY);

    // start "wav playing" LED
    digitalWrite(LED_BUILTIN_TTGO, HIGH);

    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\   
    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\   
    
    int tempi;

    // copy/format PCM data from SD Card to filter input array
    // this simulates reading a buffered ADC in real time
    for (int i = 0; i < N; ++i) // read a frame of input signal
    {

      if (root.available())
      { // some kind of byte swapped Intel Hex format
        tempi = readbyte(root, &data);
        __Temp_wav[0] = data;

        tempi = readbyte(root, &data);
        __Temp_wav[1] = data;

        outVal = (int16_t) ((__Temp_wav[0]) | (__Temp_wav[1] << 8));
        x[i] = ((float)outVal * 0.000015259) * volume;
      }
      else
      {
        root.close();
        wavPlaying = false;
        // stop "wav playing" LED
        digitalWrite(LED_BUILTIN_TTGO, LOW);
      }
    }// end frame processing of N samples

    // finished reading a frame of input samples

    // run the filter at the frame, update history
    if (LoopBack == false)
    { // no loopback. run the filter
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      dsps_fir_f32_ansi(&fir1, x, y, len);
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    else
    {
      // loopback test
      // LoopBack variable controlled over BLE
      for (int i = 0; i < N; ++i) // copy input direct to output
      {                           // skip the  filter
        y[i] = x[i];
      }
    }
    //||||||||||||||||||||||||||||||||||
    // Output to DAC
    // output a frame of filtered samples
    for (int i = 0; i < N; ++i) 
    {
      // NOTE:
      // this variable belongs to core 1 -- use the other buffer
      // If core 1 is ping, we are pong
      if (pingCore1 == true)
      { // PING time
        //convert to DAC format
        if (wavPlaying == true)
        {
          outRealPong[i] = (unsigned char) ((y[i] * 127.0) + 128.0);
        }
        else
        { // not playing wav, output a zero
          outRealPong[i] = (unsigned char) 0;
        }
      }
      else
      { // PONG time
        //convert to DAC format
        if (wavPlaying == true)
        {
          outRealPing[i] = (unsigned char) ((y[i] * 127.0) + 128.0);
        }
        else
        { // not playing wav, output a zero
          outRealPing[i] = (unsigned char) 0;
        }
      }

    }// end frame processing of N samples

    // this processing takes x msec.
    // out of 128 msec. available to the Frame
    // 125 usec * 1024 = 128 msec.
    // y% loaded
    //
    //endTEST = xthal_get_ccount();
    //totalTEST = endTEST - startTEST;
    //ESP_LOGI(TAG, "dsps_fir_f32_ansi - %f per sample for for %i coefficients,
    //%f per tap \n", cycles, fir_len, cycles / (float)fir_len);


    // end of frame processing
  } // end Arduino task loop
}
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 0000000000000000000000000000000000000000000000000000000000000
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
