/**
  Configuration file for Infineon CYT4BB project.

*/

#ifndef CONFIG_INFINEON_H
#define CONFIG_INFINEON_H

// SCH1600 sample averaging
#define AVG_FACTOR  2

// Filter settings
#define FILTER_RATE         30.0f       // Hz, LPF1 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        30.0f
#define FILTER_ACC3         30.0f
// Sensitivity settings
// Note: Sensor outputs are in g units, but treated as m/s? in the application (1g �� 9.8 m/s?)
#define SENSITIVITY_RATE1   200.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data
#define SENSITIVITY_RATE2   200.0f
#define SENSITIVITY_ACC1    25600.0f    // LSB / g, DYN1 Nominal Sensitivity for 20 bit data
#define SENSITIVITY_ACC2    25600.0f
#define SENSITIVITY_ACC3    25600.0f

// Decimation settings 1000/x
#define DECIMATION_RATE    2       // DEC5, Output sample rate decimation.
#define DECIMATION_ACC     2
// Temperature conversion
#define GET_TEMPERATURE(temp) ((temp) / 100.0f)

#endif // CONFIG_INFINEON_H 