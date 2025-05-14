ARM Subsystem – STM32F103C8T6 Communication Protocol

This project implements a custom communication protocol between two STM32 microcontrollers. The protocol uses GPIO lines, buttons, LEDs, and control signals for synchronized data transfer, all orchestrated by a timer-driven finite state machine (FSM). The system runs on an STM32F103C8T6 development board and was built in STM32CubeIDE using the STM32 HAL libraries.

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/AMihneaa/ARM-Subsystem/actions) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Features

This project implements the following features:
- Custom communication protocol between STM32 microcontrollers.
- Utilizes GPIO for data transfer and control signals.
- Timer-driven finite state machine (FSM) for synchronization.

## Run Locally

1. Clone the repository:
   ```bash
   git clone https://github.com/AMihneaa/ARM-Subsystem
   ```

2. Open with STM32CubeIDE
   - File → Open Project from File System → point to folder

3. Build & Flash
   - Compile project and flash via ST-Link or USB bootloader

### Hardware required:
- STM32F103C8T6 (Blue Pill)
- LEDs, buttons, and jumper wires (see GPIO assignments in code)
- Second MCU (optional, for testing communication)

## Protocol Logic (FSM)

FSM logic is driven by the variable Q. Each 10 ms, depending on GPIO input states (START, DAT, MOD, PREG), the FSM moves to a new state. Here's the high-level behavior:

### State 0 – Idle:
    - Waits for START or DAT button press
    - Updates status LEDs accordingly

### State 1 – DAT detected:
    - Waits for START to follow

### State 2 – Prepare to send command (CDA):
    - Configure data pins as output
    - Setup control lines
    - Move to transmission state

### State 3 – START detected:
    - Waits for DAT to follow

### State 4 – Mode decision (based on MOD):
    - MOD = 1 → Go to Read
    - MOD = 0 → Go to Write

### State 5 – Prepare for write:
    - CLK = input, ACK = output
    - Transition to actual data sending

### State 6 – Prepare for read:
    - CLK = output, ACK = input
    - Set data pins as input

### States 10-14 – CDA Transmission:
    - Sends 2-bit command via GPIO (CDA0/CDA1)
    - Synchronizes with ACK pin from receiver

### States 30-33 – Data Transmission (Write):
    - Sends 4-bit data through DATA0-DATA3
    - Sync with ACK

### States 60-63 – Data Reception (Read):
    - Receives 4-bit data from GPIO
    - Uses CLK to sample
    - ACKs back to sender

Each state updates LEDs to reflect system state and transitions automatically based on GPIO signals and conditions.

## Contributing

If you would like to contribute to this project, please fork the repository and submit a pull request. Contributions are welcome!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Future Improvements

- Implement external interrupts for event-driven transitions (EXTI_Init stub already exists)
- Add debouncing on buttons
- Expand protocol to handle more bits or multi-byte transfers
- Add UART debug logging
