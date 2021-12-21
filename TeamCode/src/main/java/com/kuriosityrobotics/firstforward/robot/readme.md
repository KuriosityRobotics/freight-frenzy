A place where notes and configurations are stored.

--Robot Config--

Control Hub Name: FTC-Dmvr

Config Name: arthurConfig

USB Devices(names):
- Webcam 1 (Left webcam)
- Webcam 2 (Forward webcam; Logitech C920)

Control Hub Portal(names):
- Expansion Hub 2
- Control Hub

Control Hub:

Motors:
- Motor Port 0(NeveRest 20 Gearmotor): fLeft
- Motor Port 1(NeveRest 20 Gearmotor): fLeft
- Motor Port 2(NeveRest 20 Gearmotor): fLeft
- Motor Port 3(NeveRest 20 Gearmotor): fLeft 

Servos:
- Servo Port 0(Servo): linkage
- Servo Port 1(Servo): pivot
- Servo Port 2(Servo): hopper

I2C Bus 0:
- Port 0(Rev Expansion Hub IMU): imu

Expansion Hub 2:

Motors:
- Motor Port 0(GoBILDA 5202/3/4 series): linearSlide
- Motor Port 1(BROKEN): 
- Motor Port 2(GoBILDA 5202/3/4 series): intake
- Motor Port 3(GoBILDA 5202/3/4 series): carousel

Servos:
- Servo Port 0(Servo): extenderLeft
- Servo Port 1(Servo): extenderRight

--OpenCV(Vision)--
Notes:
1. Remember to call .release() on Mats or else memory leeks will occur (consequence of yucky c++)
2. Please ignore reflections stuff in ManagedCamera (Consequence of EOCV not liking SwitchableCamera)

--Math--
Notes:
1. All units are in radians, unless otherwise specified
2. All units are in inches, unless otherwise specified

--Pathfollow, Sensors, basically everything--
Coordinate System:
1. (0,0) at warehouse on red side
2. positive x is going towards the blue warehouse
3. positive y is going towards the red carousel
4. 0 degrees is facing the carousel and positive angles is clockwise[add visualization soon]
5. ACME's FTC Dashboard uses the FTC system. This is why we have a normalizePose() function.

--Debug Thread--
Notes:
1. the DEBUG parameter controls whether the debug thread(and thus Filedumping/VisionDumping) is on. 
   Enable at your own risk.
2. If you turn DEBUG to true on robot, before you leave the garage, PLEASE PLEASE PLEASE remove everything 
   on the data folder of the control hub. If you don't, the control hub will die(until cache is cleared).
3. To remove cache, first run "adb shell" in terminal(folder in should be kuriosity-first-forward). 
   Then do "cd storage/self/primary/FIRST/data/". After that, type in "rm *". It should take a bit after 
   that, but after it executes, check by typing in "ls". If nothing shows up, then you're all set. Type 
   "exit" and then "adb disconnect".
   
--Opmode Tests--
After you're done with a pleb test, please add @Disabled before the @TeleOp or @Autonomous lines. 
This won't clutter up the driver station.

--TelemetryDump--
We have different ways of removing telemeters(and making sure that no Concurrent Modification Exceptions are thrown)
1. private final List<Telemeter> telemeters = Collections.synchronizedList(new ArrayList<>());
   ~ 85 ms (fastest and what currently used)
2. private final List<Telemeter> telemeters = new CopyOnWriteArrayList<>();
   ~ 106 ms
3. private final Queue<Telemeter> telemeters = new ConcurrentLinkedDeque<>();
   ~ 95 ms
