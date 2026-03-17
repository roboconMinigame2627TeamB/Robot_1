
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "cmsis_os.h"
#include "../include.h"
void RNSConfig(void *argument);
void Calculation(void *argument);
void STTChecker(void *argument);

osThreadId_t RNSConfig_Task_Handle;
osThreadId_t Calculation_Task_Handle;
osThreadId_t STTChecker_Task_Handle;

osSemaphoreId_t CalcSemaphore;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
