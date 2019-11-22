#ifndef SENSOR_H
#define SENSOR_H
#include "main.h"

typedef enum { Green = 0, Blue = 1, Orange = 2, Red = 3 } QualityState;

typedef struct {
  uint8_t calibrated;
  uint16_t Display[4];
  uint64_t CalibrationValue;
  uint16_t CurrentValue;
  ADC_HandleTypeDef *hAdc;
  QualityState Quality;
  uint16_t CallibrationSamples;
  uint16_t AirLevels[3];
} SensorStruct;

void SensorInit(SensorStruct *ptr, ADC_HandleTypeDef *hAdc,
                uint16_t *DisplayPins);
void GetSensorMeasurements(SensorStruct *ptr);
void SensorCalibrate(SensorStruct *ptr1, SensorStruct *ptr2);
void DisplaySensorState(SensorStruct *ptr);
void SetSensorAirLevels(SensorStruct *ptr, uint16_t Levels[]);
uint16_t GetAdcValue(SensorStruct *ptr);
void EvaluateSensor(SensorStruct *ptr);

#endif
