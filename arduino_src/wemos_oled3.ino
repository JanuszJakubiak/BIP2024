/* FILE:    HCuOLED_Things_Example_WeMos_OLED_Version
   DATE:    05/12/16
   VERSION: 0.1
   
REVISIONS:

12/03/15 Created version 0.1

This example shows how to use the HCOLED library with the WeMos D1 mini uOLED
shield. The sketch uses the line, rectangle and bitmap graphic functions together with 
the INVERT DrawMode to create a simple animation.

The sketch assumes that the shield with be connected to a WeMos D1 mini or mini Pro.

You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used directly for the purpose of promoting products that
directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, 
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR
LACK OF NEGLIGENCE. HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE
FOR ANY DAMAGES INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR 
CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER. */

#include "HCuOLED.h"

/* Include the standard wire library */
#include <Wire.h>



/* I2C address of the display */
#define I2C_ADD 0x3C

/* Array indexes for the X and Y coordinates */
#define X1 0
#define Y1 1
#define X2 2
#define Y2 3
#define MAX_STRING_LENGTH 6

/* Create an instance of the library for the WeMos D1 OLED shield */
HCuOLED HCuOLED(WEMOS_D1_MINI_OLED, I2C_ADD);

void setup() 
{
  Serial.begin(9600);
  /* Initialise the I2C wire library. For WeMos D1 I2C port is on pins D4 & D5 */
  Wire.begin(4, 5);

  /* Reset the display */
  HCuOLED.Reset();

  HCuOLED.SetFont(Terminal_8pt);
  HCuOLED.Cursor(0,5);
  HCuOLED.Print("HELLO!!!");
  HCuOLED.Refresh();

}

void loop() {
  if (Serial.available()) {
    HCuOLED.ClearBuffer();

    // Read a string until the end of the line
    String inputString = Serial.readStringUntil('\n');
    
    int index = 0;
    int row=0;

    while (index < inputString.length()) {
      // Find the position of the ';' character
      int endIndex = inputString.indexOf(';', index);

      String part;
      // If the character is found
      if (endIndex != -1) {
        // Extract the substring between the current index and the ';' character
        part = inputString.substring(index, endIndex);
        index = endIndex + 1;
      } else {
        part = inputString.substring(index);
        index=inputString.length();
      }
      String truncatedString = part.substring(0, MAX_STRING_LENGTH);
      displayOnOLED(truncatedString,row);
      row=row+1;
    }
    HCuOLED.Refresh();
  }
  delay(1000);
}

void displayOnOLED(String text, int row) {
  HCuOLED.Cursor(0, 20*row);
  char charArray[text.length() + 1];
  text.toCharArray(charArray, sizeof(charArray));
  HCuOLED.Print(charArray);
}
