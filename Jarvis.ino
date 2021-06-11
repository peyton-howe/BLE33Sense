/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

*  * Written by Peyton Howe
*     11 June 2021  
*/

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0


/* Includes ---------------------------------------------------------------- */
#include <PDM.h>
#include <JARVIS_inferencing.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>

#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 3

#define RED 22
#define BLUE 24
#define GREEN 23

//Time variables
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
long interval = 500;

//Compass heading variables
int heading;                          
float pitch, roll;
float magX,magY,magZ;
float xAcc, yAcc, zAcc;
float magXcomp, magYcomp;
float accXnorm, accYnorm;
float xAcc_norm, yAcc_norm;

//weather variables
float temperature, pressure, humidity, altitude;

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    if (!HTS.begin() or !BARO.begin() or !IMU.begin()) { 
      Serial.println("Failed to initialize the sensors!");
      while (1);
    }
    
    Serial.println("Edge Impulse Inferencing Demo");

    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);


//    //Magnetometer setup
//    IMU.setMagnetFS(0);  
//    IMU.setMagnetODR(8); 
//    IMU.setMagnetOffset(11.148682, 9.513550, -0.106812);           // place calibration data here
//    IMU.setMagnetSlope (0.004445, 0.004172, 0.004430);             // place calibration data here
//    
//    //Accelerometer setup
//    IMU.setAccelFS(2);
//    IMU.setAccelODR(5);
//    IMU.setAccelOffset(0.002635, -0.033756, 0.001941);             // place calibration data here
//    IMU.setAccelSlope (0.995365, 1.000206, 1.003778);              // place calibration data here
    
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }
}

//status variables to keep track of states
bool wake = false;
bool listen = false;
bool change = false;
bool isOpen = false;

void jarvis(){
    //start microphone
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    //begin classification
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    //listening for JARVIS
    while (wake == false){
      
      EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
      if (r != EI_IMPULSE_OK) {
          ei_printf("ERR: Failed to run classifier (%d)\n", r);
          return;
      }

      // determine wether to output weather data or compass
      if((change == false) && (millis() - previousMillis > interval)){
        //Serial.println("testing");
        previousMillis = millis();
        weather();
      }else if ((change == true) && (millis() - previousMillis > interval)) {
        previousMillis = millis();
        compass();
      }

      // JARVIS keyword gives RED light
      if (result.classification[0].value > 0.60){
        Serial.println("Hello Sir");
        digitalWrite(RED, LOW);
        wake = true;
        listen = true;
        delay(1000);
        previousMillis1 = millis();
      // turn off LEDs  
      }else{
        digitalWrite(RED, HIGH);
        digitalWrite(BLUE, HIGH);
        digitalWrite(GREEN, HIGH);
      }

//    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
//        // print the predictions
//        ei_printf("Predictions ");
//        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
//            result.timing.dsp, result.timing.classification, result.timing.anomaly);
//        ei_printf(": \n");
//        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//            ei_printf("    %s: %.5f\n", result.classification[ix].label,
//                      result.classification[ix].value);
//        }
//#if EI_CLASSIFIER_HAS_ANOMALY == 1
//        ei_printf("    anomaly score: %.3f\n", result.anomaly);
//#endif
//
//        print_results = 0;
//    }
  }
  //While JARVIS is listening
  while (listen == true) {
    //turn off LEDs
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, HIGH);
    digitalWrite(GREEN, HIGH);
    
    //run the classifier
    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    //determine if weather data or the compass is showing
    if((change == false) && (millis() - previousMillis > interval)){
       //Serial.println("testing");
       previousMillis = millis();
       weather();
    }else if ((change == true) && (millis() - previousMillis > interval)) {
      previousMillis = millis();
      compass();
    }

    // Open keyword gives GREEN light
    if (result.classification[1].value > 0.60){
      digitalWrite(GREEN, LOW);
      Serial.println("Open");
      if (isOpen == false) {
        Serial.println("Opening the Helmet");
        isOpen = true;
      }else if (isOpen == true) {
        Serial.println("cannot open the Helmet any further Sir");
      }
      delay(1000);
      listen = false;
      
    // Close keyword gives BLUE light  
    }else if (result.classification[3].value > 0.60){
      digitalWrite(BLUE, LOW);
      Serial.println("Close");
      if (isOpen == true) {
        Serial.println("closing the helmet");
        isOpen = false;
      }else if (isOpen == false) {
        Serial.println("cannot close the helmet any further Sir");
      }
      delay(1000);
      listen = false;

    // Switch keyword gives PURPLE light  
    }else if (result.classification[2].value > 0.60){
      digitalWrite(RED, LOW);
      digitalWrite(BLUE, LOW);
      Serial.println("Switch");
      delay(1000);
      listen = false;
      change = !change;

    }else if (millis() - previousMillis1 > 7000) {
        previousMillis1 = millis();
        digitalWrite(RED, HIGH);
        digitalWrite(BLUE, HIGH);
        digitalWrite(GREEN, HIGH);
        listen = false;
    }
  }
  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.buf_ready = 0;
  wake = false;
}
/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
//void ei_printf(const char *format, ...) {
//    static char print_buf[1024] = { 0 };
//
//    va_list args;
//    va_start(args, format);
//    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
//    va_end(args);
//
//    if (r > 0) {
//        Serial.write(print_buf);
//    }
//}

/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (record_ready == true) {
        for (int i = 0; i<bytesRead>> 1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));

    if (sampleBuffer == NULL) {
        free(inference.buffers[0]);
        free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    // optionally set the gain, defaults to 20
    //PDM.setGain(80);

    PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
    }


    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    free(sampleBuffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif

void weather() {
  //read all the sensor values
  temperature = (1.1965 * HTS.readTemperature()) - 22.891;
  humidity = HTS.readHumidity();

  pressure = (BARO.readPressure() * 10) - 1.5775;
  altitude = abs((1 - pow((pressure/1013.25), 0.190284)) * 145366.45);

  Serial.print(temperature);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(altitude);
}

void compass () {  
  //Read Magnetometer & Accelerometer values if available
  if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readMagnet(magX, magY, magZ);
  }

  //Correct the coordinate system for calculations
  magX = -magX;
  magY = -magY;
  magZ = -magZ;

  //Normalize raw accelerometer values.
  xAcc_norm = yAcc/sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);
  yAcc_norm = -xAcc/sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);

  //Calculate pitch and roll  
  pitch = -asin(yAcc_norm);
  roll = asin(xAcc_norm/cos(pitch));

  //Calculate the new tilt-compensated values
  magXcomp = magX*cos(pitch)+magZ*sin(pitch); 
  magYcomp = magX*sin(roll)*sin(pitch)+magY*cos(roll)-magZ*sin(roll)*cos(pitch);

  //Calculate heading
  heading = (180 * atan2(magYcomp, magXcomp)) / PI;

  //Correct heading to prevent negative values
  if (heading < 0) {
    heading += 360;
  }

  //Print the heading value
  Serial.print(heading);

  //Print the direction after the heading
  if ((heading >= 348.75) or (heading < 11.25)) {
    Serial.println(" N");
  }else if ((heading >= 11.25) && (heading < 33.75)){
    Serial.println(" NNE");
  }else if ((heading >= 33.75) && (heading < 56.25)){
    Serial.println(" NE");
  }else if ((heading >= 56.25) && (heading < 78.75)){
    Serial.println(" ENE");
  }else if ((heading >= 78.75) && (heading < 101.25)){
    Serial.println(" E");
  }else if ((heading >= 101.25) && (heading < 123.75)){
    Serial.println(" ESE");
  }else if ((heading >= 123.75) && (heading < 146.25)){
    Serial.println(" SE");
  }else if ((heading >= 146.25) && (heading < 168.75)){
    Serial.println(" SSE");
  }else if ((heading >= 168.75) && (heading < 191.25)){
    Serial.println(" S");
  }else if ((heading >= 191.25) && (heading < 213.75)){
    Serial.println(" SSW");
  }else if ((heading >= 213.75) && (heading < 236.25)){
    Serial.println(" SW");
  }else if ((heading >= 236.25) && (heading < 258.75)){
    Serial.println(" WSW");
  }else if ((heading >= 258.75) && (heading < 281.25)){
    Serial.println(" W");
  }else if ((heading >= 281.25) && (heading < 303.75)){
    Serial.println(" WNW");
  }else if ((heading >= 303.75) && (heading < 326.25)){
    Serial.println(" NW");
  }else if ((heading >= 326.25) && (heading < 348.75)){
    Serial.println(" NNW");
  }
    
}

void loop() {
  jarvis();
     
}
