/************************************************************************************************
 * DOIT ESP32 DevKit V1 + I2C 128x64 OLED + FreeRTOS Example
 * Development Environment: VS Code + PlatformIO
 * Author: Gengjun Wu
 * Copyright (c) 2022, gengjun.wu@gmail.com
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this list 
 *   of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this 
 *   list of conditions and the following disclaimer in the documentation and/or other 
 *   materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * References:
 * FreeRTOS I2C OLED Display Issue with ESP32 (https://esp32.com/viewtopic.php?t=8674)
 * 
 * Tutorials:
 * Interface OLED Graphic Display Module with ESP32 (https://lastminuteengineers.com/oled-display-esp32-tutorial/)
 * How to Multitask with FreeRTOS (How to Multitask with FreeRTOS)
 * Keep WiFi Connection Alive with FreeRTOS Task (https://www.youtube.com/watch?v=YSGPcm-qxDA)
************************************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Note: The SSD1306 OLED that uses I2C has default clock speed 400000UL/100000UL, in order
//       to make it work with FreeRTOS, it has to be changed to 100000UL/100000UL as below.
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEEN_HEIGHT, &Wire, -1, 100000UL, 100000UL);

int count1 = 0;
int count2 = 0;

void countTask1(void * parameters) {
  for(;;) {
    Serial.print("Current count1 is: "); // For serial monitoring purpose
    Serial.println(count1++);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay the task for about 1s
  }
}

void countTask2(void * parameters) {
  for(;;) {
    Serial.print("Current count2 is: "); // For serial monitoring purpose
    Serial.println(count2++);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay the task for about 1s
  }
}

void displayCountTask(void * parameters) {
  for(;;) {
    oled.clearDisplay(); // The OLED must be cleared before displaying
    oled.setCursor(0,0);
    oled.println(F("Current count1 is: "));
    oled.setCursor(0,18);
    oled.print(count1);
    oled.setCursor(0,35);
    oled.println(F("Current count2 is: "));
    oled.setCursor(0,53);
    oled.print(count2);
    oled.display();
    vTaskDelay(500 / portTICK_PERIOD_MS); // The delay is a bit faster than the counting to avoid missed count
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Setup the serial baudrate, this need to be set in platformio.ini file too

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed."));
    for(;;);
  }

  delay(1000);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  // Create count 1 task
  xTaskCreate(
    countTask1,
    "Counter1",
    1024,
    NULL,
    1,
    NULL
  );

  // Create count 2 task
  xTaskCreate(
    countTask2,
    "Counter2",
    1024,
    NULL,
    1,
    NULL
  );

  // Create update OLED task, the task must be pinned to the ESP32 running core, otherwise the chip will keeps rebooting itself.
  xTaskCreatePinnedToCore(
    displayCountTask,
    "OLED",
    4096,
    NULL,
    0,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );

  Serial.println("Setup completed.");
}

void loop() {
  // put your main code here, to run repeatedly:
  // oled.clearDisplay();
  vTaskDelay(100 / portTICK_PERIOD_MS); // The task delay is needed here for the OLED task to update count
  // oled.setTextSize(1);
  // oled.setTextColor(WHITE);
  // oled.setCursor(0,0);
  // oled.println(F("Current count1 is: "));
  // oled.setCursor(0,18);
  // oled.print(count1);
  // oled.setCursor(0,35);
  // oled.println(F("Current count2 is: "));
  // oled.setCursor(0,53);
  // oled.print(count2);
  // oled.display();
}