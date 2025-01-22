# Subsystems
## ElevatorMotors
### Variables
Kraken joinedMotor
CANRange rangeFinder
MagnentSensor atBottomSensor

### Methods
public ElevatorMotors()
public void configureMotors()
public double getCurrentHeight()
public void setHeight()

## CoralArm
<!-- Does coral intake require spinning gripper motors? -->

### Variables
Falcon shoulderMotor
Neo gripperMotor

### Methods
public double getShoulderAngle()
public void setShoulderAngle()

public void setGripperSpeed()
public void stopGripper()

<!-- public void intakeCoral() -->

## AlgaeArm
### Variables
Neo intakeWheelsMotor
Falcon intakePivot

### Methods
public void deployIntake()
public void retreatIntake()

public void spinIntakeWheels()
public void stopIntakeWheels()

public void intakeAlgae()
public void scoreAlgae()

## Climber
TODO!

# Supersystems
## ElevatorSupersystem
initializes: ElevatorMotors + CoralArm

<!-- Maybe algae supersystem -->