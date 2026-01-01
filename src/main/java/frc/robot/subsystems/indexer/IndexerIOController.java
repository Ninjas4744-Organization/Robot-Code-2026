package frc.robot.subsystems.indexer;

public class IndexerIOController implements IndexerIO {
    public void setPosition(Object position) {
        IndexerIO.super.setPosition(position);
    }

    public void setPercent(double percent) {
        IndexerIO.super.setPercent(percent);
    }

    public void setEncoder(double position) {
        IndexerIO.super.setEncoder(position);
    }

    public void setup() {
        IndexerIO.super.setup();
    }

    public void periodic() {
        IndexerIO.super.periodic();
    }

    public void updateInputs(IndexerIOInputsAutoLogged indexerIOInputsAutoLogged) {
        IndexerIO.super.updateInputs(indexerIOInputsAutoLogged);
    }
}
