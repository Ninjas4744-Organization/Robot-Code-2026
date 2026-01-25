package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    AddressableLED leds;
    AddressableLEDBuffer ledBuffer;

    public Leds() {
        leds = new AddressableLED(1);
        ledBuffer = new AddressableLEDBuffer(60);
        leds.setLength(ledBuffer.getLength());

        leds.setData(ledBuffer);
        leds.start();
    }
}
