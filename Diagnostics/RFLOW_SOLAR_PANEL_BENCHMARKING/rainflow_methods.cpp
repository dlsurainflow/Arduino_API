//#include "rainflow_methods.h"
//#include <arduino.h>
//
//#ifdef DEBUG_MODE
//#define DEBUG_PRINT(x) Serial.println(x)
//#else
//#define DEBUG_PRINT(x)
//#endif
//
//
//void attachRainGauage(int rainGaugePin) {
//  DEBUG_PRINT("Attaching pin : " + String(rainGaugePin));
//  pinMode(rainGaugePin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
//}
//
//void tippingBucket(double rainfallAmount, const double tippingAmount, unsigned long lastDetectedTipMillis) {
//  int tipInterval = 1000;
//  if (millis() - lastDetectedTipMillis > tipInterval) {
//    rainfallAmount += tippingAmount;
//    DEBUG_PRINT("Bucket tipped! Total Rainfall: " + String(rainfallAmount) + "mL");
//    lastDetectedTipMillis = millis();
//  }
//}
//
//void setTipAmount(double tipAmount, const double tippingAmount) {
//  tippingAmount = tipAmount;
//  DEBUG_PRINT("Tipping Amount set to: " + String(tippingAmount));
//}
