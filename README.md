# DJI C610 + M2006 interface library

## Requirements
* Platformio

## Usage

### C610Bus
Use this class if you want low-level access to up to 8 C610 controllers connected to a single CAN bus.

Example (still need to test these scripts actually):
```cpp
#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0;
C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.

void setup()
{
}
void loop()
{
    bus.PollCAN(); // Check for messages from the motors.

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {
        bus.CommandTorques(100, 200, 300, 400, C610Subbus::kIDZeroToThree);      // Command 100mA to motor 1, 200ma to motor 2, etc. The last parameter specifies to command the motors with IDs 1-4
        bus.CommandTorques(500, 600, 700, 800, C610Subbus::kIDFourToSeven);      // Command 500mA to motor 5, 600ma to motor 6, etc. The last parameter specifies to command the motors with IDs 5-8.
        int32_t m0_counts = bus.Get(0).counts();        // Get the current encoder count reading for motor 0. Returns 0 - 8191 which covers one full rotation of the motor (not to be mistaken with the output shaft).
        int32_t m1_rpm = bus.Get(1).rpm();              // Get the current rpm reading for motor 1.
        int32_t m2_approx_torque = bus.Get(2).torque(); // Get the current torque estimate for motor 2. Units are in mA (motor current is proportional to torque).

        last_command = now;
    }
}
```