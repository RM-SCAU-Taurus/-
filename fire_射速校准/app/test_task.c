#include "test_task.h"
#include "cmsis_os.h"
#include "func_generator.h"

#define TEST_PERIOD 5

//FGT_sin_t test_sin_func = 
//{
//    .Td = TEST_PERIOD,
//    .time = 0,
//    .max = 0 + 0,
//    .min = 0 - 0,
//    .dc = 0,
//    .T = 800,
//    .A = 250,
//    .phi = 0,
//    .out = 0
//};

void test_task(void const *argu)
{
    uint32_t wake_up_time = osKernelSysTick();
    for(;;)
    {
//        FGT_sin_cal(&test_sin_func);
        
        osDelayUntil(&wake_up_time, TEST_PERIOD);
    }
}
