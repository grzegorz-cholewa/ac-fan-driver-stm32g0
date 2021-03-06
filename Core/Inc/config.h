
#ifndef CONFIG_H_
#define CONFIG_H_

/* HARDWARE-DEPENDENT CONFIG */
#define NON_ISOLATED_SENSOR_NUMBER 3
#define ISOLATED_SENSOR_NUMBER 3
#define TOTAL_SENSOR_NUMBER (NON_ISOLATED_SENSOR_NUMBER + ISOLATED_SENSOR_NUMBER)
#define OUTPUT_CHANNELS_NUMBER 3

/* WORKING PARAMETERS */
#define MAIN_TIMER_RESOLUTION_US 100
#define GATE_PULSE_MIN_TIME_US 100
#define MIN_WORKING_TEMPERATURE 0*TEMPERATURE_PRECISION_MULTIPLIER // exceeding this value results in error alert
#define MAX_WORKING_TEMPERATURE 90*TEMPERATURE_PRECISION_MULTIPLIER // exceeding this value results in error alert
#define ZERO_CROSSING_DETECTION_OFFSET_US 0
#define MIN_GATE_DELAY_US 500
#define MAX_GATE_DELAY_US 9500
#define MIN_OUTPUT_VOLTAGE_DECPERCENT (35*VOLTAGE_PRECISION_MULTIPLIER) // if value is less then that, FULL_OFF_OUTPUT_VOLTAGE_DECPERCENT is set
#define MAX_OUTPUT_VOLTAGE_DECPERCENT (100*VOLTAGE_PRECISION_MULTIPLIER) // if value is bigger then that, FULL_ON_OUTPUT_VOLTAGE_DECPERCENT is set
#define FULL_OFF_OUTPUT_VOLTAGE_DECPERCENT 0
#define FULL_ON_OUTPUT_VOLTAGE_DECPERCENT (100 * VOLTAGE_PRECISION_MULTIPLIER)

/* PI PARAMETERS */
#define WORKING_PARAMETERS_UPDATE_PERIOD_US 1000000 // this is also a period of triggering PI regulator
#define INIT_CHANNEL_SETPOINT_C (0*TEMPERATURE_PRECISION_MULTIPLIER)
#define INIT_VOLTAGE (0 * VOLTAGE_PRECISION_MULTIPLIER)
#define PI_KP 5
#define TIME_CONST 10
#define INTEGRAL_ERROR_MIN 0
#define INTEGRAL_ERROR_MAX (1000*TIME_CONST)

/* RS485 PARAMETERS */
#define RS_TX_BUFFER_SIZE 100
#define RS_RX_BUFFER_SIZE 50
#define MAX_TIME_BETWEEN_FRAMES_US 4500 //((1000000/RS_BAUD_RATE)*11*4) //>((1000000/RS_BAUD_RATE)*11*4) //min 3.5 char between messages

/* CONSTANTS */
#define PI (3.14)
#define HALF_SINE_PERIOD_US 10000 // constant for 50Hz AC voltage
#define VOLTAGE_PRECISION_MULTIPLIER 10
#define TEMPERATURE_PRECISION_MULTIPLIER 10

#endif /* CONFIG_H_ */
