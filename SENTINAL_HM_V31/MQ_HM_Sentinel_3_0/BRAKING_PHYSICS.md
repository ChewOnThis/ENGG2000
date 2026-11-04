# Electrical wheel braking versus body-rate control

Checked against the [TI DRV8874 datasheet, section 7.3.2.2, Table 4](https://www.ti.com/lit/ds/symlink/drv8874.pdf) and [NASA's reaction-wheel explanation](https://science.nasa.gov/learn/basics-of-space-flight/chapter11-2/).

With nSLEEP high and PMODE latched high (PWM mode), the input combinations are:

| IN1 | IN2 | Driver state |
| --- | --- | --- |
| LOW | LOW | High-impedance coast |
| LOW | HIGH | One drive polarity |
| HIGH | LOW | Opposite drive polarity |
| HIGH | HIGH | Low-side electrical braking |

The existing motorHardware(0) writes both inputs HIGH. Despite its legacy MOTOR_IDLE/motorOff names, that is an electrical brake on the wheel, not a coast command or a guarantee that the suspended body stops. Nonzero commands alternate drive polarity with the HIGH/HIGH brake state during PWM. This assumes the established PMODE-high hardware configuration; software has not electrically verified that pin setting.

Motor torque exchanges angular momentum between the wheel and the body. Accelerating or decelerating the wheel produces an opposing body torque. A spinning wheel may be required while the Sentinel is motionless, and abruptly braking that wheel can move the Sentinel. The support string also applies external torque, so spacecraft-style conservation of total angular momentum alone is not a complete plant model.

HM 3.0's travel controller compares requested BODY rate with gyro BODY rate. It adds a signed rate-error correction and bounded load trim to the existing encoder-derived wheel-voltage feedforward. When body rate is above its request, the correction changes toward opposing body torque; the total motor voltage can stay nonzero and even retain its previous polarity because wheel back-EMF must also be accommodated. That differs from issuing motorHardware(0) whenever the body is too fast.

The feedforward conversion and BODY_SIGN/ENCODER_SIGN are empirical values retained from the working HM 2.5 setup. PWM is not a measured torque or motor current; no winding resistance, inertia or string stiffness has been newly identified. Encoder RPM is measured relative wheel motion; gyro rate measures the body. Saturation and external load can prevent the requested rate from being achieved.

Tests exercise the actual hardware mapping and signed braking correction with both wheel-speed signs and both body travel directions. These check the algorithm and pin commands, not measured physical deceleration. The successful HM 2.5 fine-hold and firing control, including its active rate damping and wheel feedforward, is preserved. Existing fixed wheel-brake phases are not reinterpreted as zero-body-speed guarantees or changed as part of this travel-only tuning.
