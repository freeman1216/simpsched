# SIMPle SCHEDuler
## What is this?
A lightweight header-only FCFS scheduler with priority for ARM mcus for event driven applications
## Features
- Priority driven scheduler
- Lightweight (200 lines of C)
- No external dependencies
- Optional dynamic parameter allocation using memory pools
## How to use it  
1. Include the header in your project  
2. Define "SIMP_SCHED_IMPLEMENTATION" and include the header 

```c
#define SIMP_SCHED_IMPLEMENTATION
#include "simpsched.h"
```
3. Define the parameter queues for the tasks and parameters that are unchanged for different iterations of the task
```c
CREATE_TASK_QUEUE(user_task, USER_TASK_QUEUE_LEN);

typedef struct {
    // static parameters here
}user_task_static_params_t ;
static const user_task_static_params_t uset_task_sparams = {//user sparams initialisation};
```
4. Registers the tasks 
```c
user_task_reg(user_task,
                  USER_TASK_PRIO, 
                  user_task_queue, 
                  USER_TASK_QUEUE_LEN, 
                  &user_task_sparams);

```
5. Run execute_scheduled() in the main loop (execute_scheduled() returns the priority of the task when it encounters an eroor (returns SCH_TASK_EXECUTION_ERROR ))
```c
    while(1){
        switch(execute_scheduled()){
            case COUNTER_TASK_PRIO: {
                //error_handler
                break;
            }

            case SCH_NO_TASKS_POSTED:{
                __dmb();
                __wfe();
                break;
            }
        }
    }
```
6. Invoke the tasks 
```c
 user_task_post(USER_TASK_PRIO, user_signal, user_params);
```
## Dynamic allocation
Xmacros are used to generate the memory pools for dynamic parameter allocation
To use them uncoment #define SIMP_SCHED_USE_PARALLOC in the header
And before including the header define 
```c
#define LIST_OF_PARAM_TYPES \
    PAR_TYPE(user_task_params_t,user_params,USER_TASK_QUEUE_LEN) 
  
```
Or if you want to use it across multiple files define them in the header itself


The main application (`src/main.c`) provides an example with dynamic allocation

You can freely modify or copy whatever you need.
