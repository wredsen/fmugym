within Credibility.Examples.SimpleControlledDriveNonlinear;

model SimpleControlledDrive_virtualMeasurement "Drive train for virtual measurements"
  extends Modelica.Icons.Example;
  extends Credibility.Examples.SimpleControlledDriveNonlinear.PartialDrive(redeclare SimpleControlledDriveNonlinear.DataVariant001 data, inertiaLoad(a(fixed = true, start = 0)), damper(d = d_exact));
  parameter Units.SI.RotationalDampingConstant d_exact = 100 "Exact value of damping constant used for virtual measurement";
  Modelica.Blocks.Continuous.LimPID speedController(k = 100, Ti = 0.1, yMax = 12, Ni = 0.1, initType = Modelica.Blocks.Types.Init.SteadyState, controllerType = Modelica.Blocks.Types.SimpleController.PI, limiter(u(start = 0)), Td = 0.1) "k, Ti, Ni: defined from control development, to be tested on control robustness ; yMax, yMin: from system spec; Init defined by optimization scenario spec" annotation(
    Placement(transformation(origin = {-46, 0}, extent = {{-90, -10}, {-70, 10}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque torqueLoad(tau_constant = 10, useSupport = false) "tau: guess value with uncertainty, could be dynamic load (boundary condition)" annotation(
    Placement(transformation(origin = {6, 0}, extent = {{102, -10}, {82, 10}})));
  Modelica.Blocks.Sources.SineVariableFrequencyAndAmplitude sine(useConstantAmplitude = true, constantAmplitude = 0.01, useConstantFrequency = false) annotation(
    Placement(transformation(extent = {{-50, 50}, {-30, 70}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 30, duration = 4, offset = 1) annotation(
    Placement(transformation(extent = {{-90, 50}, {-70, 70}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensorLoad annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {50, -40})));
  Modelica.Blocks.Interfaces.RealOutput out_motor_speed "Connector of Real output signal" annotation(
    Placement(transformation(extent = {{100, -70}, {120, -50}}), iconTransformation(extent = {{110, -60}, {110, -60}})));
  Modelica.Blocks.Interfaces.RealOutput out_load_speed "Connector of Real output signal" annotation(
    Placement(transformation(origin = {0, 6}, extent = {{100, -100}, {120, -80}}), iconTransformation(extent = {{110, -90}, {110, -90}})));
  Modelica.Blocks.Math.Add add(k2 = -1) annotation(
    Placement(transformation(origin = {-144, -40}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Interfaces.RealOutput out_control_err "Connector of Real output signal" annotation(
    Placement(transformation(origin = {-225, 20}, extent = {{100, -70}, {120, -50}}), iconTransformation(origin = {-247, -50}, extent = {{110, -60}, {110, -60}})));
  Modelica.Blocks.Math.Add add3 annotation(
    Placement(transformation(origin = {-90, 5}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Interfaces.RealInput in_torque annotation(
    Placement(transformation(origin = {-127, 27}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-146, 62}, extent = {{-20, -20}, {20, 20}})));
initial equation

equation
  connect(ramp.y, sine.f) annotation(
    Line(points = {{-69, 60}, {-60, 60}, {-60, 54}, {-52, 54}}, color = {0, 0, 127}));
  connect(sine.y, speedController.u_s) annotation(
    Line(points = {{-29, 60}, {-20, 60}, {-20, 36}, {-138, 36}, {-138, 0}}, color = {0, 0, 127}));
  connect(inertiaLoad.flange_b, torqueLoad.flange) annotation(
    Line(points = {{78, 0}, {88, 0}}));
  connect(inertiaLoad.flange_a, speedSensorLoad.flange) annotation(
    Line(points = {{58, 0}, {50, 0}, {50, -30}}, color = {0, 0, 0}));
  connect(add.y, out_control_err) annotation(
    Line(points = {{-136, -40}, {-114, -40}}, color = {0, 0, 127}));
  connect(add.u1, sine.y) annotation(
    Line(points = {{-154, -36}, {-176, -36}, {-176, 36}, {-20, 36}, {-20, 60}, {-28, 60}}, color = {0, 0, 127}));
  connect(add3.u2, speedController.y) annotation(
    Line(points = {{-100, 0}, {-114, 0}}, color = {0, 0, 127}));
  connect(in_torque, add3.u1) annotation(
    Line(points = {{-127, 27}, {-106, 27}, {-106, 10}, {-100, 10}}, color = {0, 0, 127}));
  connect(add3.y, torqueMotor.tau) annotation(
    Line(points = {{-82, 6}, {-74, 6}, {-74, 0}, {-56, 0}}, color = {0, 0, 127}));
  connect(speedSensor.w, out_motor_speed) annotation(
    Line(points = {{-8, -32}, {-8, -60}, {110, -60}}, color = {0, 0, 127}));
  connect(speedSensor.w, add.u2) annotation(
    Line(points = {{-8, -32}, {-8, -60}, {-176, -60}, {-176, -44}, {-154, -44}}, color = {0, 0, 127}));
  connect(speedSensor.w, speedController.u_m) annotation(
    Line(points = {{-8, -32}, {-76, -32}, {-76, -18}, {-126, -18}, {-126, -12}}, color = {0, 0, 127}));
  connect(speedSensorLoad.w, out_load_speed) annotation(
    Line(points = {{50, -50}, {52, -50}, {52, -84}, {110, -84}}, color = {0, 0, 127}));
  annotation(
    experiment(StopTime = 4),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {255, 0, 0}, extent = {{-96, 76}, {-24, 44}}), Text(textColor = {255, 0, 0}, extent = {{-94, 87}, {-27, 79}}, textString = "reference speed generation"), Text(origin = {26, 0}, textColor = {255, 0, 0}, extent = {{-142, 24}, {-88, 18}}, textString = "controller"), Rectangle(origin = {105, 2},lineColor = {255, 0, 0}, extent = {{-254, 16}, {-171, -16}})}),
    Documentation(info = "<html>
<p>
A&nbsp;controlled simple drive train to generate noisy signals of speed sensors.
The signals <code>noisyMotorSpeed</code> and <code>noisyLoadSpeed</code> can be
used as virtual measurements for
<a href=\"modelica://Credibility.Examples.SimpleControlledDriveNonlinear.SimpleControlledDrive_forCalibration\">model calibration</a>,
whereby this example is configured in a&nbsp;way especially suitable for calibration
of the damping parameter <code>damper.d</code>. Therefore, <code>damper.d</code>
is set to its nominal value of 100&nbsp;N&middot;m&middot;s/rad here, which is the
value a&nbsp;successful calibration shall striver.
</p>
<p>
A&nbsp;suitable reference motor speed for calibration of this drive train
configuration is prescribed by block <code>sine</code> which generates a&nbsp;sine
sweep signal of constant amplitude. It is used as a&nbsp;reference value of
the feedback controller which commands a&nbsp;motor torque for the drive train.
</p>
</html>"));
end SimpleControlledDrive_virtualMeasurement;