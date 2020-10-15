# DJI C610 + M2006 interface library

## Requirements

* Platformio

## Usage

### C610Bus

Use this class if you want low-level access to up to 8 C610 controllers connected to a single CAN bus.

Example (still need to test these scripts actually):

``` cpp
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
        // These lines will cause the motors to turn. Make sure they are mounted safely. 
        bus.CommandTorques(100, 200, 300, 400, C610Subbus::kIDZeroToThree);      // Command 100mA to motor 1, 200ma to motor 2, etc. The last parameter specifies to command the motors with IDs 1-4
        bus.CommandTorques(500, 600, 700, 800, C610Subbus::kIDFourToSeven);      // Command 500mA to motor 5, 600ma to motor 6, etc. The last parameter specifies to command the motors with IDs 5-8.
        float m0_pos = bus.Get(0).Position(); // Get the shaft position of motor 0 in radians.
        float m1_vel = bus.Get(1).Velocity(); // Get the shaft velocity of motor 1 in radians/sec.
        float m2_current = bus.Get(2).Current(); // Get the current estimate of motor 2 in amps.

        Serial.print(m0_pos);
        Serial.print(" ");
        Serial.print(m1_vel);
        Serial.print(" ");
        Serial.println(m2_current);

        last_command = now;
    }
}
```
