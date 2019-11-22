#include "sensor.h"
#include <stdint.h>
#include <string.h>

uint16_t GetAdcValue(SensorStruct *ptr) {
  uint16_t value = 0;
  HAL_ADC_Start(ptr->hAdc);
  if (HAL_ADC_PollForConversion(ptr->hAdc, 100) == HAL_OK) {
    value = HAL_ADC_GetValue(ptr->hAdc);
  }
  HAL_Delay(10);
  HAL_ADC_Stop(ptr->hAdc);
  return value;
}

void SensorInit(SensorStruct *ptr, ADC_HandleTypeDef *hAdc,
                uint16_t *DisplayPins) {
  ptr->hAdc = hAdc;
  memcpy(ptr->Display, DisplayPins, sizeof(DisplayPins) * 4);
  ptr->CallibrationSamples = 100;
}

void GetSensorMeasurements(SensorStruct *ptr) {
  ptr->CurrentValue = GetAdcValue(ptr);
}

void EvaluateSensor(SensorStruct *ptr) {
  GetSensorMeasurements(ptr);
  QualityState states[] = {Blue, Orange, Red};

  for(int t = sizeof(states)-1; t >= 0; t --) {
    if (ptr->CurrentValue > ptr->AirLevels[t]) {
      ptr->Quality = states[t];
      return;
    }
  }

  ptr->Quality = Green;
}

void CalculateLevels(SensorStruct *ptr) {
  ptr->AirLevels[0] = (ptr->CalibrationValue * 120) / 100;
  ptr->AirLevels[1] = (ptr->CalibrationValue * 140) / 100;
  ptr->AirLevels[2] = (ptr->CalibrationValue * 180) / 100;
}

void SensorCalibrate(SensorStruct *ptr1, SensorStruct *ptr2) {
  static int i = 0;
  static int j = 0;
  static int p = 0;
  p++;
  p &= 0x03;
  HAL_Delay(1000);

  if (ptr1 != NULL && ptr1->calibrated == 0) {
    if (i < ptr1->CallibrationSamples) {
      ptr1->CalibrationValue += GetAdcValue(ptr1);
      ptr1->Quality = p;
      i++;

    } else {
      ptr1->calibrated = 1;
      ptr1->CalibrationValue /= ptr1->CallibrationSamples;
      i = 0;
      CalculateLevels(ptr1);
    }
  }
  if (ptr2 != NULL && ptr2->calibrated == 0) {
    if (j < ptr2->CallibrationSamples) {
      ptr2->CalibrationValue += GetAdcValue(ptr2);
      j++;
      ptr2->Quality = p;

    } else {
      ptr2->calibrated = 1;
      ptr2->CalibrationValue /= ptr2->CallibrationSamples;
      CalculateLevels(ptr2);
      j = 0;
    }
  }
}

uint8_t CheckCalibration(SensorStruct *ptr) { return ptr->calibrated; }

void DisplaySensorState(SensorStruct *ptr) {
  uint8_t i = 0;
  for (i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(GPIOE, ptr->Display[i], GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(GPIOE, ptr->Display[ptr->Quality], GPIO_PIN_SET);
}
