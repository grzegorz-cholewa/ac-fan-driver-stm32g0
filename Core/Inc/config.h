
#ifndef CONFIG_H_
#define CONFIG_H_

/* HARDWARE-DEPENDENT CONFIG */
#define OUTPUT_CHANNELS_NUMBER 2

/* LOGGING */
#define LOGGING_PERIOD_US 5000000 // this is also a period of triggering PI regulator

/* GATE DRIVING PARAMETERS */
#define ZERO_CROSSING_DETECTION_OFFSET_US 400
#define INIT_VOLTAGE (0 * VOLTAGE_PRECISION_MULTIPLIER)
#define GATE_PULSE_MIN_TIME_US 100
#define MIN_GATE_DELAY_US 500
#define MAX_GATE_DELAY_US 9500
#define MIN_OUTPUT_VOLTAGE_DECPERCENT (35*VOLTAGE_PRECISION_MULTIPLIER) // for each smaller value gate is idle all time
#define MAX_OUTPUT_VOLTAGE_DECPERCENT (100*VOLTAGE_PRECISION_MULTIPLIER) // for each bigger value gate is active all time

/* MODBUS PARAMETERS */
#define MODBUS_TX_BUFFER_SIZE 100
#define MODBUS_RX_BUFFER_SIZE 100
#define MAX_TIME_BETWEEN_FRAMES_US 4500 //((1000000/RS_BAUD_RATE)*11*4) //>((1000000/RS_BAUD_RATE)*11*4) //min 3.5 char between messages

/* OTHER */
#define MAIN_TIMER_RESOLUTION_US 100
#define HALF_SINE_PERIOD_US 10000 // constant for 50Hz AC voltage
#define VOLTAGE_PRECISION_MULTIPLIER 10
#define TEMPERATURE_PRECISION_MULTIPLIER 10
#define PI (3.14)

#endif /* CONFIG_H_ */
