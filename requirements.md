# Software Requirements

## Subsystems

### *Drive*

#### Components
* 4 motors, two per side 
* encoder on each pair of motors
* gyro on chassis
* proximity sensor on front

#### Functions
* drive with joysticks
* drive straight when trigger held
* detect when close to switch

### *Lift*

#### Components

* 2 motors, connected together
* transmission for low and high speed
* pneumatic shifting
* encoder on pair of motors
* end-point limit switches
* ? limit switches at each height

#### Functions
* raise and lower manually
* move to preset heights
	* ground
	* switch
	* vault
	* scale
	* climb
* climb in low gear

### *Manipulator*

#### Components
* ? motors, pneumatics, etc.
* IR sensor for box detection

#### Functions
* intake box until grabbed
* spit box out of robot
* deploy / retract for storage

### *Camera*

#### Components
* one camera, mounted on manipulator

#### Functions
* stream video to driver

### *Lights*

#### Components
* multicolor LEDs

#### Functions
* flash lights when box grabbed

## Autonomous

* cross baseline (drive forward)
* drive to switch and score box
* selection based on placement
	* left
	* center
	* right