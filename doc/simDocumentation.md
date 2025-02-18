
<h1>Simulation Documentation</h1>
<h2>Motor Sim</h2>
<h3>Purpose</h3>

Motors do not update encoder values in sim, classes such as TalonFXSimState don't do any of the physics required for this

In order to make our motors function properly both in sim and on the robot, we have motor wrappers in order to simulate motor physics during sim

Motor wrappers extend the base motor class and do not affect behavior outside of sim so they are non-intrusive, only requiring the constructor to be changed. This means that Mech can just code FSMs "like normal" without having to worry about what Sim is doing

<h3>How to use</h3>

Simply switch out the base constructor for the regular motor class with our motor wrapper class, and use as normal

Example:
```java
// without motor sim: 
elevatorMotor = new TalonFX(HardwareMap.CAN_ID_ELEVATOR);

// using motor sim:
elevatorMotor = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR);
```

<h3>Technical documentation</h3>

All motor classes have some built in motor sim. All of these are partial implementations. As is shown by online resources, motor wrappers are not the intended solution to simulate behavior, 
