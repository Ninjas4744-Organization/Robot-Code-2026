package frc.robot.subsystems.indexer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.Subsystem;

public class Indexer extends SubsystemBase implements
        Subsystem.Resettable,
        Subsystem.Stoppable,
        Subsystem.Periodicable,
        Subsystem.PositionControlled<Double>,
        Subsystem.AngleControlled<Rotation2d>
{
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private boolean enabled;


    public Command setPosition(Double position) {
        return Commands.runOnce(() ->io.setPercent(position));
    }

    public Double getPosition() {
        return 0.0;
    }

    public boolean isReset() {
        return false;
    }

    public void reset() {

    }

    public Command stop() {
        return null;
    }

    public void periodic() {
        super.periodic();
    }

    public Command setAngle(Rotation2d angle) {
        return null;
    }

    public Rotation2d getAngle() {
        return null;
    }
}
