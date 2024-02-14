#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/timing/timing.h> // only for timing functions

#define TIMER_INTERVAL_MSEC 100
#define STACK_SIZE 1024
#define PRIORITY 7

K_SEM_DEFINE(sensor_sem, 0, 1);
K_SEM_DEFINE(transmit_sem, 0, 1);

static struct k_timer interrupt_timer;

struct sensor_value accel_data[3];

void timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&sensor_sem);
}

void sense_task(void *unused1, void *unused2, void *unused3)
{
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(accel0));
    if (!device_is_ready(dev)) {
        printk("Accelerometer device is not ready\n");
        return;
    }
    
    while (1)
    {
        // timing_t start_time = timing_counter_get(); #Uncomment for timing functions

        k_sem_take(&sensor_sem, K_FOREVER);

        if (sensor_sample_fetch(dev) < 0) {
            printk("Failed to fetch sample\n");
            continue;
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel_data) < 0) {
            printk("Failed to get accelerometer data\n");
            continue;
        }

        /* timing_t end_time = timing_counter_get();
        uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);
        uint64_t total_ns = timing_cycles_to_ns(total_cycles);
        printk("Sensor data acquisition took %llu ns\n", total_ns); */ //Uncomment for timing functions

        k_sem_give(&transmit_sem);
    }
}

void transmit_task(void *unused1, void *unused2, void *unused3)
{
    while (1) {
        k_sem_take(&transmit_sem, K_FOREVER);

        /* timing_t start_time = timing_counter_get(); */

        printk("Accel X: %d.%06d, Y: %d.%06d, Z: %d.%06d m/s^2\n",
               accel_data[0].val1, accel_data[0].val2,
               accel_data[1].val1, accel_data[1].val2,
               accel_data[2].val1, accel_data[2].val2);

       /*  timing_t end_time = timing_counter_get();
        uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);
        uint64_t total_ns = timing_cycles_to_ns(total_cycles);
        printk("Data transmission took %llu ns\n", total_ns); */
    }
}

void main(void) {

/* #ifdef CONFIG_TIMING_FUNCTIONS 
    timing_init();
    timing_start();
#endif */ //Uncomment for timing functions

    k_timer_init(&interrupt_timer, timer_handler, NULL);
    k_timer_start(&interrupt_timer, K_MSEC(TIMER_INTERVAL_MSEC), K_MSEC(TIMER_INTERVAL_MSEC));
}

K_THREAD_DEFINE(sense_task_id, STACK_SIZE, sense_task, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(transmit_task_id, STACK_SIZE, transmit_task, NULL, NULL, NULL, PRIORITY, 0, 0);
