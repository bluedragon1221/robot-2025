# Naming Scheme
Motor variable names: `[position or function]Motor`
    eg Elevator.leftMotor
       CoralArm.shoulderMotor
       AlgaeArm.pivotMotor

Sensor variable names: `[attribute/data]Sensor`

Method names:
    - getters: `get[motor position/function if necessary][attribute/data]`
        - bool getters: `is[condition]`
    - setters: `set[motor position/function if necessary][attribute/data]`
    - small executors: `[function][motor position/function if necessary]`
    - `configureMotors()`
    - full sequences: `seq[function][motor position/function if necessary]`
a getter should be private unless its supersystem requires that data

# Subsystems
## Elevator
### Variables
TalonFX leftMotor
TalonFX rightMotor
CANRange heightSensor
MagnentSensor atBottomSensor

### Methods
public ElevatorMotors()
public void configureMotors()

public bool isAtBottom()
public double getHeight()

public void setHeight(double height)

## CoralArm
### Variables
TalonFX pivotMotor
CANSparkMax gripperMotor

### Methods
public CoralArm()
private configureMotors()
public double getShoulderAngle()
public void setShoulderAngle()

public void setGripperSpeed()
public void stopGripper()

<!-- public void intakeCoral() -->

## AlgaeArm
### Variables
TalonFX pivotMotor
CANSparkMax gripperMotor

### Methods
public double getPivotAngle()
public void setPivotAngle(double angle)

public void setGripperSpeed(double speed)
public void stopGripper()

public void seqIntake()
public void seqIcore()
public void seqDeployIntake()
public void seqRetreatIntake()

## Climber
TODO!

# Supersystems
## ElevatorSupersystem
