package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem.ISubsystem;

import static edu.wpi.first.units.Units.Seconds;

public class Leds extends SubsystemBase implements ISubsystem.Stoppable {
    private boolean enabled;
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;

    public Leds(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            leds = new AddressableLED(1);
            ledBuffer = new AddressableLEDBuffer(60);
            leds.setLength(ledBuffer.getLength());
            leds.setData(ledBuffer);
            leds.start();
        }
    }

    public void setLedPattern(LEDPattern pattern) {
        if (enabled)
            pattern.applyTo(ledBuffer);
    }

    public void blink(Color color, double timeSec) {
        setLedPattern(LEDPattern.solid(color).blink(Seconds.of(timeSec)));
    }

    @Override
    public void stop() {
        if (enabled)
            leds.stop();
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }
}
