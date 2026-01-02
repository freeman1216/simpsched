#pragma once
#ifndef SIMP_SCHED_H

#include <stdint.h>

//Enable parameter allocation
#define SIMP_SCHED_USE_PARALLOC

typedef struct{
    
    uint32_t sig;
    void * params; 
} task_exec_info_t;


typedef enum {
    USER_TASK_PRIO_0 = 0,
    USER_TASK_PRIO_1,
    USER_TASK_PRIO_2,
    USER_TASK_PRIO_3,
    USER_TASK_MAX
} user_task_prio_t;

typedef enum {
    SCH_OK = UINT32_MAX, 
    SCH_ERR_INVALID_TASK = UINT32_MAX-1,
    SCH_ERR_INVALID_TASK_PRIO = UINT32_MAX-2,
    SCH_ERR_TASK_QUEUE_FULL = UINT32_MAX-3,
    SCH_ERR_TASK_NOT_INITIALISED = UINT32_MAX-4,
    SCH_NO_TASKS_POSTED = UINT32_MAX-5,
    SCH_TASK_EXECUTION_ERROR = UINT32_MAX-6 
}sch_status_t;

typedef sch_status_t (*task_ptr) (const void *,task_exec_info_t* );

#ifndef SIMP_SCHED_STATIC
#define SIMP_SCHED_DEF extern
#else
#define SIMP_SCHED_DEF static inline
#endif

#define CREATE_TASK_QUEUE(name,len)\
_Static_assert(len >= 2,"task queue should be number of tasks + 1");\
static task_exec_info_t name##_queue[len];


SIMP_SCHED_DEF uint32_t execute_scheduled();

SIMP_SCHED_DEF sch_status_t user_task_reg(task_ptr task, 
        user_task_prio_t  task_prio, 
        task_exec_info_t* task_queue,
        uint32_t task_queue_len,
        const void* static_params);

SIMP_SCHED_DEF sch_status_t user_task_post(user_task_prio_t task_prio,uint32_t signal, void* params);

#ifdef SIMP_SCHED_USE_PARALLOC


//add your types like this before including the header or if you wanna use this across multiple files 
//uncoment this and add them here 
// #define USER_TASK_QUEUE_LEN 3
// #define LIST_OF_PARAM_TYPES \
//     PAR_TYPE(user_task_params_t,user_params,USER_TASK_QUEUE_LEN) 
//    


#define PAR_TYPE(type,name,len) type* name##_paralloc();
LIST_OF_PARAM_TYPES
#undef PAR_TYPE

#define PAR_TYPE(type,name,len) void name##_parfree();
LIST_OF_PARAM_TYPES
#undef PAR_TYPE

#endif // SIMP_SCHED_USE_PARALLOC


static inline __attribute__((always_inline)) uint32_t __ldrex(volatile uint32_t* addr){
    uint32_t res;
    __asm__ volatile ("ldrex %0, %1" : "=r"(res): "Q"(*addr): "memory");
    return res;
}

static inline __attribute__((always_inline)) uint32_t __strex(uint32_t val,volatile uint32_t * addr){
    uint32_t res;
    __asm__ volatile ("strex %0, %2, %1" : "=&r" (res), "=Q" (*addr) : "r" (val));    
    return res;
}

static inline __attribute__((always_inline)) void __clrex(){
    __asm__ volatile ("clrex":::"memory");
}

static inline __attribute__((always_inline)) void __dmb(){
    __asm__ volatile("dmb":::"memory");
}

static inline __attribute__((always_inline)) void __wfe(){
    __asm__ volatile("wfe":::"memory");
}

static inline __attribute__((always_inline)) void __sev(){
    __asm__ volatile("sev":::"memory");
}

#ifdef SIMP_SCHED_IMPLEMENTATION


typedef struct{
    task_ptr handler;
    task_exec_info_t* params_queue;
    const void* static_params;
    uint32_t queue_len;
    volatile uint32_t head;
    uint32_t tail;
}task_ctrl_t;

static task_ctrl_t internal_task_table [USER_TASK_MAX];


SIMP_SCHED_DEF sch_status_t user_task_reg(task_ptr task, 
        user_task_prio_t task_prio, 
        task_exec_info_t *task_queue,
        uint32_t task_queue_len,
        const void* static_params)
{
    if(task_prio >= USER_TASK_MAX || task_queue_len == 0 || task_queue == 0 || task == 0 ){
        return SCH_ERR_INVALID_TASK;
    }
    task_ctrl_t *ctrl = &internal_task_table[task_prio];
    ctrl->static_params = static_params;
    ctrl->handler = task;
    ctrl->params_queue = task_queue;
    ctrl->queue_len = task_queue_len;
    return SCH_OK;
}

SIMP_SCHED_DEF sch_status_t user_task_post(user_task_prio_t task_prio, uint32_t signal, void *params){
    if(task_prio>=USER_TASK_MAX){
        return SCH_ERR_INVALID_TASK_PRIO;
    }
  
    task_ctrl_t *ctrl = &internal_task_table[task_prio];
    if( ctrl->queue_len == 0 || ctrl->params_queue == 0 || ctrl->handler == 0 ){
        return SCH_ERR_TASK_NOT_INITIALISED;
    }

    uint32_t new_head;

    do{
        new_head = __ldrex(&ctrl->head);
        
        if((++new_head) == ctrl->queue_len){
            new_head = 0;
        }

        if(new_head == ctrl->tail ){
            __clrex();
            return SCH_ERR_TASK_QUEUE_FULL;
        }
    }while(__strex(new_head,&ctrl->head));
    
    ctrl->params_queue[new_head].params = params;
    ctrl->params_queue[new_head].sig = signal;

    __dmb();
    __sev();

    return SCH_OK;
}

SIMP_SCHED_DEF uint32_t execute_scheduled(){
    for (uint32_t prio = 0; prio < USER_TASK_MAX; prio++){
        task_ctrl_t *ctrl = &internal_task_table[prio];
        
        if(ctrl->tail == ctrl->head){
            continue;
        }
        
        sch_status_t status = ctrl->handler(ctrl->static_params,&ctrl->params_queue[ctrl->tail]);
        
        if((++ctrl->tail) == ctrl->queue_len){
            ctrl->tail = 0;
        };

        if(status == SCH_TASK_EXECUTION_ERROR){
            return prio;
        }    
        return SCH_OK;
    }
    return SCH_NO_TASKS_POSTED;
}

#ifdef SIMP_SCHED_USE_PARALLOC

#define PAR_TYPE(type,name,len) \
typedef struct{                 \
    volatile uint32_t head;               \
    volatile uint32_t tail;               \
    const uint32_t queue_len;     \
    type buffer[len];           \
} name##_queue;                 \
name##_queue name##_internal_queue = {.queue_len = len};                 
LIST_OF_PARAM_TYPES
#undef PAR_TYPE

#define PAR_TYPE(type,name,len)                             \
type *  name##_paralloc(){                                  \
    uint32_t new_head;                                      \
    do{                                                     \
        new_head =__ldrex(&name##_internal_queue.head);     \
                                                            \
        if((++new_head) == name##_internal_queue.queue_len){\
           new_head = 0 ;                                   \
        }                                                   \
                                                            \
        if(new_head == name##_internal_queue.tail){         \
            __clrex();                                      \
            return 0;                                       \
        }                                                   \
                                                            \
    }while(__strex(new_head, &name##_internal_queue.head)); \
                                                            \
    __dmb();                                                \
                                                            \
    return &name##_internal_queue.buffer[new_head];         \
}                                                           \
void name##_parfree(){                                      \
    if(name##_internal_queue.head == name##_internal_queue.tail ){ \
        return;                                             \
    }                                                       \
    if((++name##_internal_queue.tail) == name##_internal_queue.queue_len){\
         name##_internal_queue.tail = 0;                    \
    };                                                      \
}  
LIST_OF_PARAM_TYPES
#undef PAR_TYPE

#endif // SIMP_SCHED_USE_PARALLOC

#endif
#endif
