/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 *
 * 创建者：林宸曳LuCkY - 2025
 
 * 说明：
 * 这是FreeRTOS的配置选项头文件
 * 有一部分比较重要或者说常用的配置选项，我都加了中文注释
 * 其他的英文注释是原来就有的，暂时先保留，如果后面有使用到那些配置的话
 * 再去添加中文注释
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/************************ 已修改内容 ******************************/
/*  configCPU_CLOCK_HZ                      定义为168000000       */
/*  configTICK_RATE_HZ                      改为1000              */
/*  configMAX_PRIORITIES                    定义为32              */
/*  configTICK_TYPE_WIDTH_IN_BITS           从64修改为32          */
/*  configUSE_TIME_SLICING                  0改为1                */
/*  configSUPPORT_STATIC_ALLOCATION         1改为0                */
/*  configTOTAL_HEAP_SIZE                   4KB改为10KB           */
/*  configUSE_TRACE_FACILITY                0改为1                */
/*  configUSE_STATS_FORMATTING_FUNCTIONS    0改为1                */
/*  configUSE_PORT_OPTIMISED_TASK_SELECTION 0改为1                */
/*  configMAX_SYSCALL_INTERRUPT_PRIORITY    0设置为10             */
/*  configCHECK_FOR_STACK_OVERFLOW          2改为0                */
/*  configQUEUE_REGISTRY_SIZE               0改为8                */
/******************************************************************/

/* FreeRTOS中断服务函数相关定义 */
#define xPortPendSVHandler          PendSV_Handler
#define vPortSVCHandler             SVC_Handler
#define xPortSysTickHandler         SysTick_Handler

/******************************************************************************/
/* Hardware description related definitions. **********************************/
/******************************************************************************/

/* 定义CPU主频, 单位: Hz, 无默认需定义 */
#define configCPU_CLOCK_HZ    ( ( unsigned long ) 168000000 )


/******************************************************************************/
/* Scheduling behaviour related definitions. **********************************/
/******************************************************************************/

/* 定义FreeRTOS系统时钟节拍频率, 单位: Hz, 无默认需定义 */
#define configTICK_RATE_HZ                         1000 //设置为1000，也就是FreeRTOS系统运行节拍为1ms

/* 1: 抢占式调度器（高优先级可以抢占低优先级）, 0: 协程式调度器, 无默认需定义 */
#define configUSE_PREEMPTION                       1

/* 1: 使能时间片调度, 默认: 1 */
#define configUSE_TIME_SLICING                     1

/* 1: 使用硬件计算下一个要运行的任务,任务优先级有上限（通常为32） 0: 使用软件算法计算下一个要运行的任务,不限制任务优先级的最大值 默认: 0 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    0

/* 1: 使能tickless低功耗模式, 默认: 0 */
#define configUSE_TICKLESS_IDLE                    0

/* FreeRTOS可配置优先级数量，优先级范围 0 ~ (configMAX_PRIORITIES - 1), 无默认需定义*/
/* 对于FreeRTOS来说，优先级数字越大，优先级越高，与STM32相反，需注意 */
#define configMAX_PRIORITIES                       32

/* 定义空闲任务的栈大小, 单位: 字Word, 无默认需定义 */
#define configMINIMAL_STACK_SIZE                   128

/* 定义任务名称的最大长度（字符数）, 默认: 16 */
#define configMAX_TASK_NAME_LEN                    16

/* Time is measured in 'ticks' - which is the number of times the tick interrupt
 * has executed since the RTOS kernel was started.
 * The tick count is held in a variable of type TickType_t.
 *
 * configTICK_TYPE_WIDTH_IN_BITS controls the type (and therefore bit-width) of
 * TickType_t:
 *
 * Defining configTICK_TYPE_WIDTH_IN_BITS as TICK_TYPE_WIDTH_16_BITS causes
 * TickType_t to be defined (typedef'ed) as an unsigned 16-bit type.
 *
 * Defining configTICK_TYPE_WIDTH_IN_BITS as TICK_TYPE_WIDTH_32_BITS causes
 * TickType_t to be defined (typedef'ed) as an unsigned 32-bit type.
 *
 * Defining configTICK_TYPE_WIDTH_IN_BITS as TICK_TYPE_WIDTH_64_BITS causes
 * TickType_t to be defined (typedef'ed) as an unsigned 64-bit type. */
/* STM32定义为32位 */
#define configTICK_TYPE_WIDTH_IN_BITS              TICK_TYPE_WIDTH_32_BITS

/* 1: 使能在抢占式调度下,同优先级的任务能抢占空闲任务, 默认: 1 */
#define configIDLE_SHOULD_YIELD                    1

/* 定义任务通知数组的大小, 默认: 1 */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES      1

/* 定义可以注册的信号量和消息队列的个数, 默认: 0 */
#define configQUEUE_REGISTRY_SIZE                  8

/* 1: 兼容老版本, 默认: 1 */
#define configENABLE_BACKWARD_COMPATIBILITY        0

/* 定义线程本地存储指针的个数, 默认: 0 */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS    0

/* When configUSE_MINI_LIST_ITEM is set to 0, MiniListItem_t and ListItem_t are
 * both the same. When configUSE_MINI_LIST_ITEM is set to 1, MiniListItem_t
 * contains 3 fewer fields than ListItem_t which saves some RAM at the cost of
 * violating strict aliasing rules which some compilers depend on for
 * optimization. Defaults to 1 if left undefined. */
#define configUSE_MINI_LIST_ITEM                   1

/* 定义任务堆栈深度的数据类型, 默认: size_t  */
#define configSTACK_DEPTH_TYPE                     size_t

 /* 定义消息缓冲区中消息长度的数据类型, 默认: size_t */
#define configMESSAGE_BUFFER_LENGTH_TYPE           size_t

/* If configHEAP_CLEAR_MEMORY_ON_FREE is set to 1, then blocks of memory
 * allocated using pvPortMalloc() will be cleared (i.e. set to zero) when freed
 * using vPortFree(). Defaults to 0 if left undefined. */
#define configHEAP_CLEAR_MEMORY_ON_FREE            1

/* vTaskList and vTaskGetRunTimeStats APIs take a buffer as a parameter and
 * assume that the length of the buffer is configSTATS_BUFFER_MAX_LENGTH.
 * Defaults to 0xFFFF if left undefined. New applications are recommended to use
 * vTaskListTasks and vTaskGetRunTimeStatistics APIs instead and supply the
 * length of the buffer explicitly to avoid memory corruption. */
#define configSTATS_BUFFER_MAX_LENGTH              0xFFFF

/* 1: 任务创建时分配Newlib的重入结构体, 默认: 0 */
#define configUSE_NEWLIB_REENTRANT                 0




/******************************************************************************/
/* Software timer related definitions. ****************************************/
/******************************************************************************/

/* 1: 使能软件定时器, 默认: 0 */
#define configUSE_TIMERS                1

/* 定义软件定时器任务的优先级, 无默认。 configUSE_TIMERS为1时需定义 */
#define configTIMER_TASK_PRIORITY       ( configMAX_PRIORITIES - 1 )

/* 定义软件定时器任务的栈空间大小, 无默认。 configUSE_TIMERS为1时需定义 */
#define configTIMER_TASK_STACK_DEPTH    (configMINIMAL_STACK_SIZE * 2)

/* 定义软件定时器命令队列的长度, 无默认。 configUSE_TIMERS为1时需定义 */
#define configTIMER_QUEUE_LENGTH        5




/******************************************************************************/
/* Event Group related definitions. *******************************************/
/******************************************************************************/

/* Set configUSE_EVENT_GROUPS to 1 to include event group functionality in the
 * build. Set to 0 to exclude event group functionality from the build. The
 * FreeRTOS/source/event_groups.c source file must be included in the build if
 * configUSE_EVENT_GROUPS is set to 1. Defaults to 1 if left undefined. */

#define configUSE_EVENT_GROUPS    1

/******************************************************************************/
/* Stream Buffer related definitions. *****************************************/
/******************************************************************************/

/* Set configUSE_STREAM_BUFFERS to 1 to include stream buffer functionality in
 * the build. Set to 0 to exclude event group functionality from the build. The
 * FreeRTOS/source/stream_buffer.c source file must be included in the build if
 * configUSE_STREAM_BUFFERS is set to 1. Defaults to 1 if left undefined. */

#define configUSE_STREAM_BUFFERS    1

/******************************************************************************/
/* Memory allocation related definitions. *************************************/
/******************************************************************************/

/* 1: 支持静态申请内存，在编译时预分配所有资源提高确定性和安全性, 默认: 0 */
#define configSUPPORT_STATIC_ALLOCATION              0

/* 1: 支持动态申请内存，运行时动态灵活分配资源，但如果资源不足可能会导致程序跑飞, 默认: 1 */
#define configSUPPORT_DYNAMIC_ALLOCATION             1

/* FreeRTOS堆中可用的RAM总量, 单位: Byte, 无默认需定义 */
#define configTOTAL_HEAP_SIZE                        (16 * 1024)

/* 1: 用户手动分配FreeRTOS内存堆(ucHeap), 默认: 0 */
#define configAPPLICATION_ALLOCATED_HEAP             0

/* 1: 用户自行实现任务创建时使用的内存申请与释放函数, 默认: 0 */
/* FreeRTOS自带的函数为pvPortMallocStack() vPortFreeStack() */
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP    0

/* Set configENABLE_HEAP_PROTECTOR to 1 to enable bounds checking and
 * obfuscation to internal heap block pointers in heap_4.c and heap_5.c to help
 * catch pointer corruptions. Defaults to 0 if left undefined. */
#define configENABLE_HEAP_PROTECTOR                  0




/******************************************************************************/
/* 中断相关配置                                                                */
/******************************************************************************/

/* Cortex-M内核使用8bit配置优先级，STM32使用了8bit中的高4位 */
/* 也就是__NVIC_PRIO_BITS的值是4 */
#define configPRIO_USEDBITS 4


/* =============================== SysTick中断优先级配置 =============================== */

/* FreeRTOS的内核中断配置，因为使用SysTick作为时钟源，所以这里其实是在配置SysTick的优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15                  /* STM32的中断最低优先级 */
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_USEDBITS) )


/*===========================================可屏蔽的中断优先级配置====================================================*/
/* 一个中断函数如果调用 FreeRTOS API 函数，那么这个中断的优先级应该 <= configMAX_SYSCALL_INTERRUPT_PRIORITY
   也就是抢占优先级编号要大于等于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5                   /* FreeRTOS可管理的最高中断优先级 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_USEDBITS) )

/* configMAX_SYSCALL_INTERRUPT_PRIORITY 的另一个名字 */
#define configMAX_API_CALL_INTERRUPT_PRIORITY    configMAX_SYSCALL_INTERRUPT_PRIORITY


/******************************************************************************/
/* Hook and callback function related definitions. 钩子函数相关  ***************/
/******************************************************************************/

#define configUSE_IDLE_HOOK                   0     /* 1: 使能空闲任务钩子函数, 无默认需定义  */
#define configUSE_TICK_HOOK                   0     /* 1: 使能系统时钟节拍中断钩子函数, 无默认需定义 */
#define configUSE_MALLOC_FAILED_HOOK          0     /* 1: 使能动态内存申请失败钩子函数, 默认: 0。启用可提升调试能力，但需实现vApplicationMallocFailedHook() */
#define configUSE_DAEMON_TASK_STARTUP_HOOK    0     /* 1: 使能定时器服务任务首次执行前的钩子函数, 默认: 0 */

/* Set configUSE_SB_COMPLETED_CALLBACK to 1 to have send and receive completed
 * callbacks for each instance of a stream buffer or message buffer. When the
 * option is set to 1, APIs xStreamBufferCreateWithCallback() and
 * xStreamBufferCreateStaticWithCallback() (and likewise APIs for message
 * buffer) can be used to create a stream buffer or message buffer instance
 * with application provided callbacks. Defaults to 0 if left undefined. */
#define configUSE_SB_COMPLETED_CALLBACK       0

/* 0：不使用, 1: 使能栈溢出检测方法1, 2: 使能栈溢出检测方法2, 默认: 0 */
#define configCHECK_FOR_STACK_OVERFLOW        0

/******************************************************************************/
/* Run time and task stats gathering related definitions. *********************/
/******************************************************************************/

/* 1: 使能任务运行时间统计功能, 默认: 0 */
/* 如果设置这个值为1，那么下面两个函数需要被定义 
portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()：用户程序需要提供一个基准时钟函数，
函数完成初始化基准时钟功能，并define到宏portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()上。
需要一个比系统节拍中断频率还要高分辨率的基准定时器，否则，统计可能不精确。
基准定时器中断频率要比统节拍中断快10~100倍。越快越精准，但能统计的运行时间也越短

portGET_RUN_TIME_COUNTER_VALUE()：用户程序需要提供一个返回基准时钟当前“时间”的函数，
这个函数要被define到宏portGET_RUN_TIME_COUNTER_VALUE()上。*/
#define configGENERATE_RUN_TIME_STATS           0

/* 1: 使能可视化跟踪调试, 默认: 0 */
#define configUSE_TRACE_FACILITY                1

/* 0：不编译 1：编译vTaskList()和vTaskGetRunTimeStats(), 默认: 0 * */
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

/******************************************************************************/
/* Co-routine related definitions. ********************************************/
/******************************************************************************/

/* 1: 启用协程式调度, 默认: 0 */
#define configUSE_CO_ROUTINES              0

/* 定义协程的最大优先级, 最大优先级=configMAX_CO_ROUTINE_PRIORITIES-1, 无默认。configUSE_CO_ROUTINES为1时需定义 */
/*增大允许更细粒度优先级管理；减小简化调度器，至少需1*/
#define configMAX_CO_ROUTINE_PRIORITIES    2

/******************************************************************************/
/* Debugging assistance. ******************************************************/
/******************************************************************************/

/* configASSERT() has the same semantics as the standard C assert().  It can
 * either be defined to take an action when the assertion fails, or not defined
 * at all (i.e. comment out or delete the definitions) to completely remove
 * assertions.  configASSERT() can be defined to anything you want, for example
 * you can call a function if an assert fails that passes the filename and line
 * number of the failing assert (for example, "vAssertCalled( __FILE__, __LINE__
 * )" or it can simple disable interrupts and sit in a loop to halt all
 * execution on the failing line for viewing in a debugger. */
#define configASSERT( x )         \
    if( ( x ) == 0 )              \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for( ; ; )                \
        ;                         \
    }

/******************************************************************************/
/* FreeRTOS MPU specific definitions. *****************************************/
/******************************************************************************/

/* If configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS is set to 1 then
 * the application writer can provide functions that execute in privileged mode.
 * See:
 * https://www.freertos.org/a00110.html#configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS
 * Defaults to 0 if left undefined.  Only used by the FreeRTOS Cortex-M MPU
 * ports, not the standard ARMv7-M Cortex-M port. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS    0

/* Set configTOTAL_MPU_REGIONS to the number of MPU regions implemented on your
 * target hardware.  Normally 8 or 16.  Only used by the FreeRTOS Cortex-M MPU
 * ports, not the standard ARMv7-M Cortex-M port.  Defaults to 8 if left
 * undefined. */
#define configTOTAL_MPU_REGIONS                                   8

/* configTEX_S_C_B_FLASH allows application writers to override the default
 * values for the for TEX, Shareable (S), Cacheable (C) and Bufferable (B) bits
 * for the MPU region covering Flash.  Defaults to 0x07UL (which means TEX=000,
 * S=1, C=1, B=1) if left undefined.  Only used by the FreeRTOS Cortex-M MPU
 * ports, not the standard ARMv7-M Cortex-M port. */
#define configTEX_S_C_B_FLASH                                     0x07UL

/* configTEX_S_C_B_SRAM allows application writers to override the default
 * values for the for TEX, Shareable (S), Cacheable (C) and Bufferable (B) bits
 * for the MPU region covering RAM. Defaults to 0x07UL (which means TEX=000,
 * S=1, C=1, B=1) if left undefined.  Only used by the FreeRTOS Cortex-M MPU
 * ports, not the standard ARMv7-M Cortex-M port. */
#define configTEX_S_C_B_SRAM                                      0x07UL

/* Set configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY to 0 to prevent any privilege
 * escalations originating from outside of the kernel code itself.  Set to 1 to
 * allow application tasks to raise privilege.  Defaults to 1 if left undefined.
 * Only used by the FreeRTOS Cortex-M MPU ports, not the standard ARMv7-M
 * Cortex-M port. */
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY               1

/* Set configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS to 1 to allow unprivileged
 * tasks enter critical sections (effectively mask interrupts). Set to 0 to
 * prevent unprivileged tasks entering critical sections.  Defaults to 1 if left
 * undefined.  Only used by the FreeRTOS Cortex-M MPU ports, not the standard
 * ARMv7-M Cortex-M port. */
#define configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS                0

/* FreeRTOS Kernel version 10.6.0 introduced a new v2 MPU wrapper, namely
 * mpu_wrappers_v2.c. Set configUSE_MPU_WRAPPERS_V1 to 0 to use the new v2 MPU
 * wrapper. Set configUSE_MPU_WRAPPERS_V1 to 1 to use the old v1 MPU wrapper
 * (mpu_wrappers.c). Defaults to 0 if left undefined. */
#define configUSE_MPU_WRAPPERS_V1                                 0

/* When using the v2 MPU wrapper, set configPROTECTED_KERNEL_OBJECT_POOL_SIZE to
 * the total number of kernel objects, which includes tasks, queues, semaphores,
 * mutexes, event groups, timers, stream buffers and message buffers, in your
 * application. The application will not be able to have more than
 * configPROTECTED_KERNEL_OBJECT_POOL_SIZE kernel objects at any point of
 * time. */
#define configPROTECTED_KERNEL_OBJECT_POOL_SIZE                   10

/* When using the v2 MPU wrapper, set configSYSTEM_CALL_STACK_SIZE to the size
 * of the system call stack in words. Each task has a statically allocated
 * memory buffer of this size which is used as the stack to execute system
 * calls. For example, if configSYSTEM_CALL_STACK_SIZE is defined as 128 and
 * there are 10 tasks in the application, the total amount of memory used for
 * system call stacks is 128 * 10 = 1280 words. */
#define configSYSTEM_CALL_STACK_SIZE                              128

/* When using the v2 MPU wrapper, set configENABLE_ACCESS_CONTROL_LIST to 1 to
 * enable Access Control List (ACL) feature. When ACL is enabled, an
 * unprivileged task by default does not have access to any kernel object other
 * than itself. The application writer needs to explicitly grant the
 * unprivileged task access to the kernel objects it needs using the APIs
 * provided for the same. Defaults to 0 if left undefined. */
#define configENABLE_ACCESS_CONTROL_LIST                          1

/******************************************************************************/
/* SMP( Symmetric MultiProcessing ) Specific Configuration definitions. *******/
/******************************************************************************/

/* Set configNUMBER_OF_CORES to the number of available processor cores.
 * Defaults to 1 if left undefined. */

/*
 #define configNUMBER_OF_CORES                     [Num of available cores]
 */

/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
 * configRUN_MULTIPLE_PRIORITIES to 0 to allow multiple tasks to run
 * simultaneously only if they do not have equal priority, thereby maintaining
 * the paradigm of a lower priority task never running if a higher priority task
 * is able to run. If configRUN_MULTIPLE_PRIORITIES is set to 1, multiple tasks
 * with different priorities may run simultaneously - so a higher and lower
 * priority task may run on different cores at the same time. */
#define configRUN_MULTIPLE_PRIORITIES             0

/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
 * configUSE_CORE_AFFINITY to 1 to enable core affinity feature. When core
 * affinity feature is enabled, the vTaskCoreAffinitySet and
 * vTaskCoreAffinityGet APIs can be used to set and retrieve which cores a task
 * can run on. If configUSE_CORE_AFFINITY is set to 0 then the FreeRTOS
 * scheduler is free to run any task on any available core. */
#define configUSE_CORE_AFFINITY                   0

/* When using SMP with core affinity feature enabled, set
 * configTASK_DEFAULT_CORE_AFFINITY to change the default core affinity mask for
 * tasks created without an affinity mask specified. Setting the define to 1
 * would make such tasks run on core 0 and setting it to (1 <<
 * portGET_CORE_ID()) would make such tasks run on the current core. This config
 * value is useful, if swapping tasks between cores is not supported (e.g.
 * Tricore) or if legacy code should be controlled. Defaults to tskNO_AFFINITY
 * if left undefined. */
#define configTASK_DEFAULT_CORE_AFFINITY          tskNO_AFFINITY

/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), if
 * configUSE_TASK_PREEMPTION_DISABLE is set to 1, individual tasks can be set to
 * either pre-emptive or co-operative mode using the vTaskPreemptionDisable and
 * vTaskPreemptionEnable APIs. */
#define configUSE_TASK_PREEMPTION_DISABLE         0

/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
 * configUSE_PASSIVE_IDLE_HOOK to 1 to allow the application writer to use
 * the passive idle task hook to add background functionality without the
 * overhead of a separate task. Defaults to 0 if left undefined. */
#define configUSE_PASSIVE_IDLE_HOOK               0

/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one),
 * configTIMER_SERVICE_TASK_CORE_AFFINITY allows the application writer to set
 * the core affinity of the RTOS Daemon/Timer Service task. Defaults to
 * tskNO_AFFINITY if left undefined. */
#define configTIMER_SERVICE_TASK_CORE_AFFINITY    tskNO_AFFINITY

/******************************************************************************/
/* ARMv8-M secure side port related definitions. ******************************/
/******************************************************************************/

/* secureconfigMAX_SECURE_CONTEXTS define the maximum number of tasks that can
 *  call into the secure side of an ARMv8-M chip.  Not used by any other ports.
 */
#define secureconfigMAX_SECURE_CONTEXTS        5

/* Defines the kernel provided implementation of
 * vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Idle task and Timer task
 * respectively. The application can provide it's own implementation of
 * vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory() by
 * setting configKERNEL_PROVIDED_STATIC_MEMORY to 0 or leaving it undefined. */
#define configKERNEL_PROVIDED_STATIC_MEMORY    1

/******************************************************************************/
/* ARMv8-M port Specific Configuration definitions. ***************************/
/******************************************************************************/

/* Set configENABLE_TRUSTZONE to 1 when running FreeRTOS on the non-secure side
 * to enable the TrustZone support in FreeRTOS ARMv8-M ports which allows the
 * non-secure FreeRTOS tasks to call the (non-secure callable) functions
 * exported from secure side. */
#define configENABLE_TRUSTZONE            1

/* If the application writer does not want to use TrustZone, but the hardware
 * does not support disabling TrustZone then the entire application (including
 * the FreeRTOS scheduler) can run on the secure side without ever branching to
 * the non-secure side. To do that, in addition to setting
 * configENABLE_TRUSTZONE to 0, also set configRUN_FREERTOS_SECURE_ONLY to 1. */
#define configRUN_FREERTOS_SECURE_ONLY    1


/*是否启用 MPU（Memory Protection Unit，内存保护单元） 1：启用 0：不启用*/
/*启用后，可对任务访问的内存区域进行限制，提高系统安全性和稳定性*/
#define configENABLE_MPU                  1

/*是否启用 FPU（浮点运算单元）支持。 1：启用 0：不启用*/
/*启用后，FreeRTOS会在任务切换时保存/恢复浮点寄存器，如果任务中有较多的浮点数计算，则应该启用，且需要定义STM32的启用FPU*/
#define configENABLE_FPU                  1

/* Set configENABLE_MVE to 1 to enable the M-Profile Vector Extension (MVE)
 * support, or 0 to leave the MVE support disabled. This option is only
 * applicable to Cortex-M55 and Cortex-M85 ports as M-Profile Vector Extension
 * (MVE) is available only on these architectures. configENABLE_MVE must be left
 * undefined, or defined to 0 for the Cortex-M23,Cortex-M33 and Cortex-M35P
 * ports. */
#define configENABLE_MVE                  1

/******************************************************************************/
/* ARMv7-M and ARMv8-M port Specific Configuration definitions. ***************/
/******************************************************************************/

/* Set configCHECK_HANDLER_INSTALLATION to 1 to enable additional asserts to
 * verify that the application has correctly installed FreeRTOS interrupt
 * handlers.
 *
 * An application can install FreeRTOS interrupt handlers in one of the
 * following ways:
 *   1. Direct Routing  -  Install the functions vPortSVCHandler and
 * xPortPendSVHandler for SVC call and PendSV interrupts respectively.
 *   2. Indirect Routing - Install separate handlers for SVC call and PendSV
 *                         interrupts and route program control from those
 * handlers to vPortSVCHandler and xPortPendSVHandler functions. The
 * applications that use Indirect Routing must set
 * configCHECK_HANDLER_INSTALLATION to 0.
 *
 * Defaults to 1 if left undefined. */
#define configCHECK_HANDLER_INSTALLATION    1

/******************************************************************************/
/* Definitions that include or exclude functionality. *************************/
/******************************************************************************/

/* Set the following configUSE_* constants to 1 to include the named feature in
 * the build, or 0 to exclude the named feature from the build. */
#define configUSE_TASK_NOTIFICATIONS           1    /* 1: 使能任务间直接的消息传递,包括信号量、事件标志组和消息邮箱, 默认: 1 */
#define configUSE_MUTEXES                      1    /* 1: 使能互斥信号量（互斥锁）, 默认: 0 */
#define configUSE_RECURSIVE_MUTEXES            1    /* 1: 使能递归互斥信号量（递归互斥锁）, 默认: 0 */
#define configUSE_COUNTING_SEMAPHORES          1    /* 1: 使能计数信号量, 默认: 0 */
#define configUSE_QUEUE_SETS                   0    /* 1: 使能队列集, 默认: 0 */
#define configUSE_APPLICATION_TASK_TAG         0    

/* USE_POSIX_ERRNO enables the task global FreeRTOS_errno variable which will
 * contain the most recent error for that task. */
#define configUSE_POSIX_ERRNO                  0

/* 0：失能函数 1：使能函数*/
#define INCLUDE_vTaskPrioritySet               1    //设置任务优先级
#define INCLUDE_uxTaskPriorityGet              1    //获取任务优先级
#define INCLUDE_vTaskDelete                    1    //删除任务
#define INCLUDE_vTaskSuspend                   1    //挂起任务
#define INCLUDE_vTaskDelayUntil                1    //任务绝对延时
#define INCLUDE_vTaskDelay                     1    //任务延时
#define INCLUDE_xTaskGetSchedulerState         1    //获取任务调度器状态
#define INCLUDE_xTaskGetCurrentTaskHandle      1    //获取当前任务的句柄
#define INCLUDE_uxTaskGetStackHighWaterMark    1    //获取任务堆栈历史剩余最小值
#define INCLUDE_xTaskGetIdleTaskHandle         1    //获取空闲任务的句柄
#define INCLUDE_eTaskGetState                  1    //获取任务状态
#define INCLUDE_xTimerPendFunctionCall         1    //将函数的执行挂到定时器服务任务
#define INCLUDE_xTaskAbortDelay                1    //中断任务延时
#define INCLUDE_xTaskGetHandle                 1    //通过任务名获取任务句柄
#define INCLUDE_xTaskResumeFromISR             1    //恢复在中断中挂起的任务

#endif /* FREERTOS_CONFIG_H */
