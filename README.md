## Avnet PSOC™ Edge DEEPCRAFT™ Motion

This demo project is the integration of Infineon's [PSOC&trade; Edge MCU: Machine learning – DEEPCRAFT™ deploy motion](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-deploy-motion/tree/release-v2.0.0)
and [Avnet /IOTCONNECT ModusToolbox&trade; SDK](https://github.com/avnet-iotconnect/avnet-iotc-mtb-sdk). 

This code example demonstrates how to deploy a machine learning (ML) model to detect motion generated using DEEPCRAFT&trade; Studio on Infineon's PSOC&trade; Edge MCU.

This example deploys DEEPCRAFT&trade; Studio's human activity detection starter model on the PSOC&trade; Edge MCU. This model uses data from the BMI270 IMU to detect various motions such as standing, running, walking, sitting, and jumping.

This project has a three project structure: CM33 secure, CM33 non-secure, and CM55 projects. All three projects are programmed to the external QSPI flash and executed in Execute in Place (XIP) mode. Extended boot launches the CM33 secure project from a fixed location in the external flash, which then configures the protection settings and launches the CM33 non-secure application. Additionally, CM33 non-secure application enables CM55 CPU and launches the CM55 application.

The M55 processor performs the DEEPCRAFT™ model heavy lifting and reports the data via IPC to the M33 processor.
The M33 Non-Secure application is a custom /IOTCONNECT application that is receiving the IPC messages, 
processing the data and sending it to /IOTCONNECT. 
This application can receive Cloud-To-Device commands as well and control one of the board LEDs or control the application flow.    

## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.6 or later (tested with v3.6)
- Board support package (BSP) minimum required version: 1.0.0
- Programming language: C
- Associated parts: All [PSOC&trade; Edge MCU](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm) parts

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`

## Supported kits (make variable 'TARGET')

- [PSOC&trade; Edge E84 AI Kit](https://www.newark.com/infineon/kitpse84aitobo1/ai-eval-kit-32bit-arm-cortex-m55f/dp/49AM4459) (`KIT_PSE84_AI`)
- [PSOC&trade; Edge E84 Evaluation Kit](https://www.newark.com/infineon/kitpse84evaltobo1/eval-kit-32bit-arm-cortex-m55f/dp/49AM4460) (`KIT_PSE84_EVAL_EPC2`)

## Hardware setup

This example uses the board's default configuration. 
See the kit user guide to ensure that the board is configured correctly.

Ensure the following jumper and pin configuration on board.
- BOOT SW must be in the HIGH/ON position
- J20 and J21 must be in the tristate/not connected (NC) position

> **Note:** This hardware setup is not required for KIT_PSE84_AI.

## Setup The Project

To setup the project, please refer to the 
[/IOTCONNECT ModusToolbox&trade; PSOC Edge Developer Guide](DEVELOPER_GUIDE.md)

## Running The Demo

- Once the board connects to /IOTCONNECT, it will begin sending telemetry packets similar to the example below:
```
>: {"d":[{"d":{"version":"1.1.0","random":32,"confidence":86,"class_id":1,"class":"standing","event_detected":true}}]}
```

- While holding the kit board in your hand, perform various activities 
such as *sitting, standing, walking, running, or jumping* 
and observe if the model detects and reports the correct activity.


- For proper detection, the sensor on the board must be oriented in the same general way in which it is trained. 
The orientation for *sitting* (KitProg USB facing forward with EVK towards the ground) and *standing* (KitProg USB facing forward with EVK towards the body). 
For *walking, running, and jumping*, hold the board in the same orientation as standing with the normal arm movements associated with the activity.


- The following commands can be sent to the device using the /IOTCONNECT Web UI:

    | Command                  | Argument Type     | Description                                                        |
    |:-------------------------|-------------------|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
    | `board-user-led`         | String (on/off)   | Turn the board LED on or off                                                                                                                                                |
    | `set-reporting-interval` | Number (eg. 2000) | Set telemetry reporting interval in milliseconds.  By default, the application will report every 2000ms                                     |
