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
#define SENSITIVITY_RATE1   6400.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data
#define SENSITIVITY_RATE2   6400.0f
#define SENSITIVITY_ACC1    25600.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data
#define SENSITIVITY_ACC2    25600.0f
#define SENSITIVITY_ACC3    25600.0f

// Decimation settings 1000/x
#define DECIMATION_RATE    1       // DEC5, Output sample rate decimation.
#define DECIMATION_ACC     1
// Temperature conversion
#define GET_TEMPERATURE(temp) ((temp) / 100.0f)

#endif // CONFIG_INFINEON_H 