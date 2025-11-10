// stm32f407_real_time_kernel.c
// Real-time multi-tasking kernel with priority-based scheduling
// Context switching and inter-task communication

#include <stdint.h>

// Task states
#define TASK_READY      0x01
#define TASK_RUNNING    0x02
#define TASK_BLOCKED    0x03
#define TASK_SUSPENDED  0x04

// Context switch addresses
#define KERNEL_BASE     0x20000000

typedef struct {
    // Register context
    uint32_t r0, r1, r2, r3, r12;
    uint32_t lr, pc, psr;
    
    // Additional registers
    uint32_t r4, r5, r6, r7;
    uint32_t r8, r9, r10, r11;
    
    uint32_t sp;
} RegisterContext;

typedef struct {
    uint32_t task_id;
    uint32_t priority;
    uint32_t state;
    
    RegisterContext *context;
    uint32_t stack_base;
    uint32_t stack_size;
    uint32_t stack_ptr;
    
    void (*task_function)(void);
    
    uint32_t cpu_time_ms;
    uint32_t wake_time;
    uint32_t time_slice_remaining;
    
    uint32_t next_task_id;
} TaskControlBlock;

typedef struct {
    uint32_t resource_id;
    uint32_t owner_task_id;
    uint32_t waiting_list[16];
    uint32_t waiting_count;
    uint32_t lock_count;
} Mutex;

typedef struct {
    uint32_t queue_id;
    uint32_t *buffer;
    uint32_t head;
    uint32_t tail;
    uint32_t size;
    uint32_t count;
    uint32_t waiting_tasks[8];
} MessageQueue;

// Global kernel state
static volatile struct {
    TaskControlBlock *current_task;
    TaskControlBlock *ready_queue[256];  // 256 priority levels
    uint32_t ready_queue_size;
    
    uint32_t kernel_ticks;
    uint32_t context_switches;
    
    uint32_t load_cpu;
    uint32_t idle_counter;
} kernel_state = {0};

// Task creation
uint32_t task_create(uint32_t priority, void (*task_func)(void), 
                    uint32_t stack_size, uint32_t task_id) {
    
    if (priority > 255) return 0xFFFFFFFF;
    
    // Allocate TCB
    static TaskControlBlock tcb_pool[32];
    static uint32_t tcb_count = 0;
    
    if (tcb_count >= 32) return 0xFFFFFFFF;
    
    TaskControlBlock *tcb = &tcb_pool[tcb_count];
    
    tcb->task_id = task_id;
    tcb->priority = priority;
    tcb->state = TASK_READY;
    tcb->task_function = task_func;
    tcb->stack_size = stack_size;
    tcb->cpu_time_ms = 0;
    tcb->time_slice_remaining = 10;  // 10ms timeslice
    
    // Add to ready queue
    kernel_state.ready_queue[priority] = tcb;
    
    tcb_count++;
    
    return task_id;
}

// Context switch function (called from SysTick)
void context_switch() {
    
    // Save current context
    if (kernel_state.current_task) {
        kernel_state.current_task->state = TASK_READY;
    }
    
    // Find next ready task (highest priority)
    TaskControlBlock *next_task = NULL;
    
    for (int p = 255; p >= 0; p--) {
        if (kernel_state.ready_queue[p] != NULL) {
            next_task = kernel_state.ready_queue[p];
            break;
        }
    }
    
    if (next_task == NULL) {
        // Idle task
        return;
    }
    
    // Switch to next task
    kernel_state.current_task = next_task;
    kernel_state.current_task->state = TASK_RUNNING;
    
    kernel_state.context_switches++;
}

// Mutex operations
uint32_t mutex_create(uint32_t resource_id) {
    
    static Mutex mutex_pool[32];
    static uint32_t mutex_count = 0;
    
    if (mutex_count >= 32) return 0xFFFFFFFF;
    
    Mutex *mtx = &mutex_pool[mutex_count];
    mtx->resource_id = resource_id;
    mtx->owner_task_id = 0xFFFFFFFF;
    mtx->lock_count = 0;
    
    return mutex_count++;
}

uint32_t mutex_lock(uint32_t mutex_id) {
    
    static Mutex mutex_pool[32];
    Mutex *mtx = &mutex_pool[mutex_id];
    
    if (mtx->owner_task_id == kernel_state.current_task->task_id) {
        // Already owned by this task (recursive)
        mtx->lock_count++;
        return 0;
    }
    
    if (mtx->owner_task_id == 0xFFFFFFFF) {
        // Mutex is free
        mtx->owner_task_id = kernel_state.current_task->task_id;
        mtx->lock_count = 1;
        return 0;
    }
    
    // Mutex is held by another task - block
    kernel_state.current_task->state = TASK_BLOCKED;
    mtx->waiting_list[mtx->waiting_count++] = kernel_state.current_task->task_id;
    
    // Force context switch
    context_switch();
    
    return 1;
}

uint32_t mutex_unlock(uint32_t mutex_id) {
    
    static Mutex mutex_pool[32];
    Mutex *mtx = &mutex_pool[mutex_id];
    
    if (mtx->owner_task_id != kernel_state.current_task->task_id) {
        return 0xFFFFFFFF;  // Error: not owner
    }
    
    mtx->lock_count--;
    
    if (mtx->lock_count == 0) {
        
        if (mtx->waiting_count > 0) {
            // Wake up next waiting task
            uint32_t waiting_task = mtx->waiting_list[0];
            
            for (int i = 0; i < mtx->waiting_count - 1; i++) {
                mtx->waiting_list[i] = mtx->waiting_list[i + 1];
            }
            mtx->waiting_count--;
            
            mtx->owner_task_id = waiting_task;
            mtx->lock_count = 1;
            
        } else {
            mtx->owner_task_id = 0xFFFFFFFF;
        }
    }
    
    return 0;
}

// Message queue operations
uint32_t queue_create(uint32_t queue_id, uint32_t *buffer, uint32_t size) {
    
    static MessageQueue queue_pool[16];
    static uint32_t queue_count = 0;
    
    if (queue_count >= 16) return 0xFFFFFFFF;
    
    MessageQueue *q = &queue_pool[queue_count];
    q->queue_id = queue_id;
    q->buffer = buffer;
    q->size = size;
    q->head = 0;
    q->tail = 0;
    q->count = 0;
    
    return queue_count++;
}

uint32_t queue_send(uint32_t queue_id, uint32_t message) {
    
    static MessageQueue queue_pool[16];
    MessageQueue *q = &queue_pool[queue_id];
    
    if (q->count >= q->size) {
        return 0xFFFFFFFF;  // Queue full
    }
    
    q->buffer[q->tail] = message;
    q->tail = (q->tail + 1) % q->size;
    q->count++;
    
    return 0;
}

uint32_t queue_receive(uint32_t queue_id, uint32_t *message, uint32_t timeout_ms) {
    
    static MessageQueue queue_pool[16];
    MessageQueue *q = &queue_pool[queue_id];
    
    if (q->count == 0) {
        
        // Queue is empty, block task
        if (timeout_ms == 0) {
            return 0xFFFFFFFF;
        }
        
        kernel_state.current_task->state = TASK_BLOCKED;
        kernel_state.current_task->wake_time = kernel_state.kernel_ticks + timeout_ms;
        
        q->waiting_tasks[q->count] = kernel_state.current_task->task_id;
        
        context_switch();
        
        return 1;  // Blocked
    }
    
    *message = q->buffer[q->head];
    q->head = (q->head + 1) % q->size;
    q->count--;
    
    return 0;
}

// Task delay
void task_delay_ms(uint32_t ms) {
    
    kernel_state.current_task->state = TASK_BLOCKED;
    kernel_state.current_task->wake_time = kernel_state.kernel_ticks + ms;
    
    context_switch();
}

// Get kernel statistics
void get_kernel_stats(uint32_t *total_tasks, uint32_t *context_switches, uint32_t *cpu_load) {
    
    *total_tasks = 32;  // Max tasks
    *context_switches = kernel_state.context_switches;
    *cpu_load = kernel_state.load_cpu;
}

// SysTick ISR (called every 1ms)
void systick_handler() {
    
    kernel_state.kernel_ticks++;
    
    // Update CPU load
    kernel_state.load_cpu = 100 - ((kernel_state.idle_counter * 100) / 1000);
    kernel_state.idle_counter = 0;
    
    // Check for tasks to wake up
    if (kernel_state.current_task && 
        kernel_state.current_task->state == TASK_BLOCKED &&
        kernel_state.kernel_ticks >= kernel_state.current_task->wake_time) {
        
        kernel_state.current_task->state = TASK_READY;
    }
    
    // Decrement timeslice
    if (kernel_state.current_task) {
        kernel_state.current_task->time_slice_remaining--;
        
        if (kernel_state.current_task->time_slice_remaining == 0) {
            kernel_state.current_task->time_slice_remaining = 10;
            context_switch();
        }
    }
}

// Idle task (runs when no other tasks ready)
void idle_task() {
    
    while (1) {
        kernel_state.idle_counter++;
        
        // WFI - Wait For Interrupt
        __asm("wfi");
    }
}
