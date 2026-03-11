package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class ShooterSubsystem  extends ActionableSubsystem {
    private double    m_rpmLeft;
    private double    m_rpmRight;
    private TalonFX   m_leftFlywheel;
    private TalonFX   m_rightFlywheel;
    private TalonFXS  m_leftFeedWheel;
    private TalonFXS  m_rightFeedWheel;
    private CANBus    m_shooterBus;
    private CANrange  m_canrange;

     // --- Shooter state machine States ---
    private enum ShooterState {
      IDLE,                         // Motors stopped, or placed in coast mode to spin down on their own.
      GOING_TO_TARGET_RPM,          // Spinning up (or down) to RPM
      WAITING_FOR_SINGLE_SHOT,      // ramping to speed; change state to FIRING_ONE and turn on
                                    // left feed motor and bed rollers (inward) when ready
      WAITING_FOR_CONTINUOUS_FIRE,  // ramping to speed; change state to FIRING_CONTINUOUS and turn
                                    // on bed rollers and feed wheels when ready
      READY_TO_FIRE,                // At Target RPM, but not triggered to fire
      FIRING_ONE,                   // Feed motor and bedmotors are on, waiting for "Fuel shot" sensor.
                                    // On sensor event, change state to READ_TO_FIRE, turn off feed motor
      FIRING_CONTINUOUS             // Keep all motors running, stop via manual control (Bumper release)
    }

    private enum ShootMode {
      ONE,
      CONTINUOOUS
    }

    private ShooterState  m_currentShooterState = ShooterState.IDLE;
    private double        m_targetFlywheelVel = 0.0;                  // Start out stopped 

    public ShooterSubsystem(CANBus shooterBus) {
        m_shooterBus = shooterBus;
        m_leftFlywheel   = new TalonFX(SSC.LEFT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_rightFlywheel  = new TalonFX(SSC.RIGHT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_leftFeedWheel  = new TalonFXS(SSC.LEFT_FEED_MOTOR_ID, m_shooterBus);
        m_rightFeedWheel = new TalonFXS(SSC.RIGHT_FEED_MOTOR_ID, m_shooterBus);
        m_canrange       = new CANrange(SSC.CANRANGE_ID, m_shooterBus);
        configFlywheels();
        configFeedMotors();
        configCANRange();
        m_currentShooterState = ShooterState.IDLE;
    }
    
    public void changeFlywheelTargetVel(double vel) {
      // Filter for reasonable values, and clamp if needed
      if (vel > SSC.MAX_FLY_VEL) vel = SSC.MAX_FLY_VEL;
      if (vel < SSC.MIN_FLY_VEL) vel = SSC.MIN_FLY_VEL;
      
      // Set the member variable to latest target - needed for increment and decrement, if used
      m_targetFlywheelVel = vel;

      // Now set the Velocity PID with the new target velocity, 
      // smd set the shooterState to GOING_TO_TARGET_RPM
      // Note that if singleShot() and fireContinuous() methods
      // need to call this method, they will immediately overwrite 
      // the state after return.  
      VelocityVoltage request = new VelocityVoltage(m_targetFlywheelVel);
      m_leftFlywheel.setControl(request);
      m_rightFlywheel.setControl(request);
      m_currentShooterState = ShooterState.GOING_TO_TARGET_RPM;
    }

    // These next two methods are not really needed. Instead, bind the
    // changeFlywheelTargetVel() method to the same buttons, passing in
    // the respective constants used here to set the speed.
    public void spinUpFlywheelClose() {
        changeFlywheelTargetVel(SSC.FLY_MOTOR_NEAR_DIST_VEL);
    }

    public void spinUpFlywheelFar() {
        changeFlywheelTargetVel(SSC.FLY_MOTOR_FAR_DIST_VEL);
    }

    // The folllowing two methods could be useful in tuning - not so much for
    // competition. Maybe bind to D-Pad buttons, but only during testing - uses too
    // many button resources... 
    public void incrementFlywheelVel() {
      changeFlywheelTargetVel(m_targetFlywheelVel + 100.0);
    }

    public void decrementFlywheelVel() {
      changeFlywheelTargetVel(m_targetFlywheelVel - 100.0);
    }

    // Bind this to the button that fires a signle shot (Y?)
    public void singleShot() {
      if (m_currentShooterState != ShooterState.READY_TO_FIRE) {
        changeFlywheelTargetVel(m_targetFlywheelVel);
        // The above method sets state to RAMPING_TO_RPM. 
        // Override that by setting to WAITING_FOR_SINGLE_SHOT
        m_currentShooterState = ShooterState.WAITING_FOR_SINGLE_SHOT;
      } else {
        startShooting(ShootMode.ONE);
      }
    }
        
    // Bind this method to the button that whileHeld
    public void shootContinuous() {
      if (m_currentShooterState != ShooterState.READY_TO_FIRE) {
        changeFlywheelTargetVel(m_targetFlywheelVel);
        // The above method sets state to RAMPING_TO_RPM. 
        // Override that by setting to WAITING_FOR_CONTINUOUS_FIRE
        m_currentShooterState = ShooterState.WAITING_FOR_CONTINUOUS_FIRE;
      } else {
        startShooting(ShootMode.CONTINUOOUS);
      }
    }

    private void startShooting(ShootMode shootMode) {
      // No speed filters here - assume fully vetted shooter readiness,
      // so double check that is correct before calling!
      if (shootMode == ShootMode.ONE) {
          startLeftFeedMotor();;
          m_currentShooterState = ShooterState.FIRING_ONE;
      } else if (shootMode == ShootMode.CONTINUOOUS) {
          startLeftFeedMotor();
          startRightFeedMotor();
          m_currentShooterState = ShooterState.FIRING_CONTINUOUS;
      } else {
          System.out.println("Invalid mode requested in ShooterSubsystem.startShooting()");
      }
    }

    private void startLeftFeedMotor() {
        // ensure bed rollers are on, inward
        VelocityVoltage request = new VelocityVoltage(SSC.FEED_MOTOR_TARGET_VEL);   //  Rot per secSSSC
        m_leftFeedWheel.setControl(request);
    }
    
    private void startRightFeedMotor() {
        // ensure bed rollers are on, inward
        VelocityVoltage request = new VelocityVoltage(SSC.FEED_MOTOR_TARGET_VEL);
        m_rightFeedWheel.setControl(request);
    }
    
    // The stopShooting() method stops both feed motors, but leaves the shooter flywheel motors
    // running, and changes state to READY_TO_SHOOT. 
    // To sop the shooter motors too (in coast mode) call shutdownShooter instead.
    // Bind the stopShooting() method to the release of whichever button is assigned to 
    // continuous shooting (ALT - R_Bumper). "WhileHeld" continues shooting.
    public void stopShooting() {
        m_leftFeedWheel.stopMotor();
        m_rightFeedWheel.stopMotor();
        // Let bed rollers continue to run?
        m_currentShooterState = ShooterState.READY_TO_FIRE;
    }

    // Provide separate getters() for areShootersReady() and isLeftShooterReady()
    private boolean areShootersReady() {
        return (isWithinTolerance(m_rightFlywheel.getVelocity().getValueAsDouble())
                && isLeftShooterReady());
    }
    
    private boolean isLeftShooterReady() {
      return isWithinTolerance(m_leftFlywheel.getVelocity().getValueAsDouble());
    }

    // isWithinTolerance() is a  helper method that uses a constant toleerance
    // (from SSC), and returns true if and only if the passed in flywheelVel velocity
    // matches the global member variable targetVel within that tolerance.
    // All units are rotations per second.
    private boolean isWithinTolerance(double vel) {
        // Inclusive check: lowerBound <= value <= upperBound
        double lowerBound  = m_targetFlywheelVel - SSC.FLY_MOTOR_VEL_TOLERANCE;
        double upperBound  = m_targetFlywheelVel + SSC.FLY_MOTOR_VEL_TOLERANCE;
        return (vel >= lowerBound && vel <= upperBound);
    }

    private boolean isFuelAtLeftShooterSensor() {
      return (m_canrange.getDistance().getValueAsDouble() <= SSC.CANRANGE_FUEL_PRESENT_THRESHOLD);
    }

    public void shutdownShooter() {
      stopShooting();                // Stops feed motors
      m_leftFlywheel.stopMotor();
      m_rightFlywheel.stopMotor();
      m_currentShooterState = ShooterState.IDLE;
    }
    
    private void runShooterStateMachine() {
        switch(m_currentShooterState) {
          case IDLE:
            // Nothing to do
            break;

          case GOING_TO_TARGET_RPM:
            if (areShootersReady()) {
              m_currentShooterState = ShooterState.READY_TO_FIRE;
            }
            break;

          case WAITING_FOR_SINGLE_SHOT:
            if (isFuelAtLeftShooterSensor()) {
              m_leftFeedWheel.stopMotor();
              if (isLeftShooterReady()) {
                // enssure bed rollers on
                startLeftFeedMotor();
                m_currentShooterState = ShooterState.FIRING_ONE;
              }
            } else {
              startLeftFeedMotor();
            }
            break;
    
          case WAITING_FOR_CONTINUOUS_FIRE:
            if (areShootersReady()) {
              startLeftFeedMotor();
              startRightFeedMotor();
              m_currentShooterState = ShooterState.FIRING_CONTINUOUS;
            }
            break;

          case READY_TO_FIRE:
            if (! areShootersReady()) {
              // the following method changes state to GOING_TO_TARGET_RPM
              changeFlywheelTargetVel(m_targetFlywheelVel);
            }
            break;

          case FIRING_ONE:
            if (! isFuelAtLeftShooterSensor()) {
              stopShooting();
            }
            break;

          case FIRING_CONTINUOUS:
            // Nothing to do, except might want to have a time limit.
            // This state should be started via a whileHeld function, so when button is
            // released, firing should be stopped and state changed in the method
            // which that release is bound to (typ. stopShooting()).
            break;

          default:
            System.out.println("Inalid state in runShooterSateMachine()" + m_currentShooterState.toString());
        }
    }

    public void update() {
      // refresh all Phoenix 6.0Pro StatusSignals here
    }

    public void periodic(){
      runShooterStateMachine();
      publishShooterData();
    }

    private void configFlywheels() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FLY_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FLY_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(1)
                                                  .withRotorToSensorRatio(SSC.FLY_GEAR_RATIO);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.FLY_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SSC.FLY_LEFT_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SSC.FLY_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SSC.FLY_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SSC.FLY_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(SSC.FLY_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SSC.FLY_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SSC.FLY_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SSC.FLY_MOTOR_KP)
                                                     .withKI(SSC.FLY_MOTOR_KI)
                                                     .withKD(SSC.FLY_MOTOR_KD)
                                                     .withKS(SSC.FLY_MOTOR_KS)
                                                     .withKV(SSC.FLY_MOTOR_KV)
                                                     .withKA(SSC.FLY_MOTOR_KA)
                                                     .withKG(SSC.FLY_MOTOR_KG);
        var leftFlyConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig)
                                                      .withCurrentLimits(currentLimitConfig)
                                                      .withOpenLoopRamps(openLoopConfig)
                                                      .withClosedLoopRamps(closedLoopConfig)
                                                      .withSlot0(pid0Configs);
        var rightFlyConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig.withInverted(SSC.FLY_RIGHT_MOTOR_INVERT))
                                                       .withCurrentLimits(currentLimitConfig)
                                                       .withOpenLoopRamps(openLoopConfig)
                                                       .withClosedLoopRamps(closedLoopConfig)
                                                       .withSlot0(pid0Configs);
        
        StatusCode status = m_leftFlywheel.getConfigurator().apply(leftFlyConfig);

        if (! status.isOK()) System.out.println("Left Flywheel motor config: "
                                                +status.getDescription());
        StatusCode statusRight = m_rightFlywheel.getConfigurator().apply(rightFlyConfig);

        if (! statusRight.isOK()) System.out.println("Right Flyhweel motor config "
                                                     +statusRight.getDescription());
    }

    private void configFeedMotors() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FEED_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FEED_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        FeedbackConfigs feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SSC.FEED_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.FEED_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SSC.LEFT_FEED_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SSC.FEED_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SSC.FEED_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(SSC.FEED_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                                            .withSupplyCurrentLimitEnable(SSC.FEED_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                                            .withStatorCurrentLimit(SSC.FEED_STATOR_CURRENT_LIMIT)
                                                                            .withStatorCurrentLimitEnable(SSC.FEED_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SSC.FEED_MOTOR_KP)
                                                     .withKI(SSC.FEED_MOTOR_KI)
                                                     .withKD(SSC.FEED_MOTOR_KD)
                                                     .withKS(SSC.FEED_MOTOR_KS)
                                                     .withKV(SSC.FEED_MOTOR_KV)
                                                     .withKA(SSC.FEED_MOTOR_KA);
        CommutationConfigs commutationConfigs = new CommutationConfigs().withAdvancedHallSupport(SSC.FEED_ADVANCED_HALL_SUPPORT_VALUE)
                                                                        .withMotorArrangement(SSC.FEED_MOTOR_ARRANGEMENT_VALUE);
        var leftFeedConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs)
                                                          .withCommutation(commutationConfigs);
        var rightFeedConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig.withInverted(SSC.FLY_RIGHT_MOTOR_INVERT))
                                                         .withCurrentLimits(currentLimitConfig)
                                                         .withOpenLoopRamps(openLoopConfig)
                                                         .withClosedLoopRamps(closedLoopConfig)
                                                         .withSlot0(pid0Configs)
                                                         .withCommutation(commutationConfigs);

        StatusCode status = m_leftFeedWheel.getConfigurator().apply(leftFeedConfig);
        if (! status.isOK()) System.out.println("Left Feed motor config: "
                                                +status.getDescription());
        
        StatusCode statusRight = m_rightFeedWheel.getConfigurator().apply(rightFeedConfig);
        if (! statusRight.isOK()) System.out.println("Right Feed Motor config "
                                                    +statusRight.getDescription());

    }
    
    private void configCANRange() {
       FovParamsConfigs fovParamsConfig = new FovParamsConfigs().withFOVCenterX(SSC.CANRANGE_FOV_CENTER_X_ANGLE)
                                                                .withFOVCenterY(SSC.CANRANGE_FOV_CENTER_Y_ANGLE)
                                                                .withFOVRangeX(SSC.CANRANGE_FOV_RANGE_X_ANGLE)
                                                                .withFOVRangeY(SSC.CANRANGE_FOV_RANGE_Y_ANGLE);
        ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs().withMinSignalStrengthForValidMeasurement(SSC.CANRANGE_MINIMUM_SIGNAL)
                                                                                    .withProximityHysteresis(SSC.CANRANGE_PROXIMITY_HYSTERESIS)
                                                                                    .withProximityThreshold(SSC.CANRANGE_FUEL_PRESENT_THRESHOLD);
        ToFParamsConfigs tofParamsConfigs = new ToFParamsConfigs().withUpdateFrequency(SSC.CANRANGE_UPDATE_FREQUENCY)
                                                                  .withUpdateMode(SSC.CANRANGE_UPDATE_MODE);
        var canrangeConfig = new CANrangeConfiguration().withFovParams(fovParamsConfig)
                                                        .withProximityParams(proximityParamsConfigs)
                                                        .withToFParams(tofParamsConfigs);
        StatusCode status = m_canrange.getConfigurator().apply(canrangeConfig);
            if (! status.isOK()) System.out.println("CANrange config "
                                                    +status.getDescription());
    }  

    // This method displays shooter data to provide the drive team / programmer a window 
    // into the ShooterSubsystem operation
    private void publishShooterData() {
      m_rpmLeft = m_leftFlywheel.getVelocity().getValueAsDouble()*60;
      m_rpmRight = m_rightFlywheel.getVelocity().getValueAsDouble()*60;
      SmartDashboard.putNumber("Left RPM", m_rpmLeft);
      SmartDashboard.putNumber("Right RPM", m_rpmRight);
      var m_currentLeft = m_leftFlywheel.getSupplyCurrent().getValueAsDouble();
      var m_currentRight = m_rightFlywheel.getSupplyCurrent().getValueAsDouble();
      SmartDashboard.putNumber("Left Current", m_currentLeft);
      SmartDashboard.putNumber("Right Current", m_currentRight);
    }

    // The distancetoRPS conversion method is a placeholder for when we have range
    // estimates of the distance between the robot's current Pose and the Pose of the 
    // target goal. It should return desired rotations per second for that distance.
    @SuppressWarnings("unused")
    private double distanceToRPS(double distance) {
        return distance; // TODO: make the equation, either through a lookup table, pure math, or interpolation.
    }
}