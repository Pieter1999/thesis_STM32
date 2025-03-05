# thesis_STM32
As part of my final master's thesis, I configured an STM32 Nucleo development board to act as the on-board computer of my simulated spacecraft as found in my project lunar_CubeSat. Code is written in C, designed for use in STM32CubeIDE.  

An overview of the most relevant files is given below:
- **Src/main.c**: Main run file based on UART reception callback function from the Hardware Abstraction Layer library. Within the callback, messages are received character by character and added to a buffer that is then converted into required floating point numbers (double). After this, reference attitude / quaternion, control torque and thrust output values are computed and communicated via UART again. In addition, based on the thrust output values, PWM signals for different pre-set timers on the Nucleo board are activated.
- **Src/func.c**: Contains the constants and functions needed for computations.
- **Inc/func.h**: Header file for all self-written functions.
- **Inc/global.h**: Header file for all self-written global variables and constants.

The remaining files are part of the automatically generated STM32CubeIDE environment for application to the STM32F303RETX configuration. Note that more back-up files, including the ecos installation, are needed for proper execution.
