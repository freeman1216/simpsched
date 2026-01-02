#define BAD_USART_IMPLEMENTATION
#define BAD_FLASH_IMPLEMENTATION
#define BAD_RCC_IMPLEMENTATION
#define BAD_GPIO_IMPLEMENTATION

#define BAD_SYSTICK_SYSTICK_ISR_IMPLEMENTATION
#define BAD_HARDFAULT_USE_UART
#define BAD_HARDFAULT_ISR_IMPLEMENTATION
#include "badhal.h"


typedef struct {
    uint32_t value;
} counter_params_t;

typedef struct {
    volatile USART_typedef_t * uart;
}counter_static_params_t ;


#define COUNTER_QUEUE_LEN 4
#define LIST_OF_PARAM_TYPES \
     PAR_TYPE(counter_params_t,counter_params,COUNTER_QUEUE_LEN)

#define SIMP_SCHED_IMPLEMENTATION
#include "simpched.h"

#define COUNTER_TASK_PRIO USER_TASK_PRIO_0
CREATE_TASK_QUEUE(counter, COUNTER_QUEUE_LEN);

#define COUNTER_USART USART1
#define COUNTER_USART_SETTINGS (USART_FEATURE_TRANSMIT_EN)
static const counter_static_params_t counter_sparams = {.uart = USART1};

#define UART_GPIO_PORT          (GPIOA)
#define UART1_TX_PIN            (9)
#define UART1_RX_PIN            (10)
#define UART1_TX_AF             (7)
#define UART1_RX_AF             (7)

    // HSE  = 25
    // PLLM = 25
    // PLLN = 400
    // PLLQ = 10
    // PLLP = 4
    // Sysclock = 100

#define BADHAL_PLLM (25)
#define BADHAL_PLLN (400)
#define BADHAL_PLLQ (10)
#define BADHAL_FLASH_LATENCY (FLASH_LATENCY_3ws)

#define BAD_RTOS_AHB1_PERIPEHRALS    (RCC_AHB1_GPIOA|RCC_AHB1_DMA2|RCC_AHB1_GPIOB)
#define BAD_RTOS_APB2_PERIPHERALS    (RCC_APB2_USART1|RCC_APB2_SPI1|RCC_APB2_SYSCFGEN|RCC_APB2_TIM10)

static inline void __main_clock_setup(){
    rcc_enable_hse();
    rcc_pll_setup( PLLP4, BADHAL_PLLM, BADHAL_PLLN, BADHAL_PLLQ, PLL_SOURCE_HSE);
    rcc_bus_prescalers_setup(HPRE_DIV_1, PPRE_DIV_2, PPRE_DIV_1);
    flash_acceleration_setup(BADHAL_FLASH_LATENCY, FLASH_DCACHE_ENABLE, FLASH_ICACHE_ENABLE);
    rcc_enable_and_switch_to_pll();
}


static inline void __periph_setup(){
    rcc_set_ahb1_clocking(BAD_RTOS_AHB1_PERIPEHRALS);
    io_setup_pin(UART_GPIO_PORT, UART1_TX_PIN, MODER_af, UART1_TX_AF, OSPEEDR_high_speed, PUPDR_no_pull, OTYPR_push_pull);
    io_setup_pin(UART_GPIO_PORT, UART1_RX_PIN, MODER_af, UART1_RX_AF, OSPEEDR_high_speed, PUPDR_no_pull, OTYPR_push_pull);
    rcc_set_apb2_clocking(BAD_RTOS_APB2_PERIPHERALS);
}

static inline void __uart_setup(){
    uart_disable(COUNTER_USART);
    uart_setup(COUNTER_USART, USART_BRR_115200,COUNTER_USART_SETTINGS , 0, 0);
    uart_enable(COUNTER_USART);
}

static inline void __systick_setup(){
    systick_setup(CLOCK_SPEED/1000, SYSTICK_FEATURE_CLOCK_SOURCE|SYSTICK_FEATURE_TICK_INTERRUPT);
    SCB_set_priority_grouping(SCB_PRIO_GROUP4);
    SCB_set_core_interrupt_priority(SCB_SYSTICK_INTR, SCB_PRIO15);
    systick_enable();
}


static volatile uint32_t ticks;

void systick_usr(){
    ticks++;
    counter_params_t * iter_params = counter_params_paralloc();
    
    if(!iter_params){
        return;
    }

    iter_params->value = ticks;
    user_task_post(COUNTER_TASK_PRIO, 0, iter_params);
}


sch_status_t counter_task(const void * sparams, task_exec_info_t * dparams){    
    counter_params_t * iter_params = dparams->params;
    const counter_static_params_t * spar = sparams;
    uart_send_str_polling(spar->uart, "executed task with this value (tick posted) ");
    uart_send_dec_unsigned_32bit(spar->uart, iter_params->value);
    counter_params_parfree();
    return SCH_OK;
}


int __attribute__((noinline)) main(){
    __DISABLE_INTERUPTS;
    
    __main_clock_setup();
    __periph_setup();
    __systick_setup();
    __uart_setup();
    
    user_task_reg(counter_task,
                  COUNTER_TASK_PRIO, 
                  counter_queue, 
                  COUNTER_QUEUE_LEN, 
                  &counter_sparams);
    __ENABLE_INTERUPTS;

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
    return 0;
}
