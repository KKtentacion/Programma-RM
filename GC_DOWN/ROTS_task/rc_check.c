#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "rc_check.h"


/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartCheckTask(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	
  for(;;)
  {
				
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}