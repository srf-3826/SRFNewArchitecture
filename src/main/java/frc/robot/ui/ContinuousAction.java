package frc.robot.ui;

public interface ContinuousAction { 
    void start();
    void update();
    void stop();
    
    default boolean isFinished() {
        return false;
    }
}