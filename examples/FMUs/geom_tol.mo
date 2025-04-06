model geom_tol
  Modelica.Blocks.Interfaces.RealOutput FR1_min annotation(
    Placement(transformation(origin = {84, 114}, extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput FR1_max annotation(
    Placement(transformation(origin = {84, 76}, extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput R1(start=0.1) annotation(
    Placement(transformation(origin = {-142, 154}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-62, 90}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput S2(start=0.1) annotation(
    Placement(transformation(origin = {-140, 111}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-54, -5}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput C4(start=0.1) annotation(
    Placement(transformation(origin = {-140, 69}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-24, -169}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput S4(start=0.2) annotation(
    Placement(transformation(origin = {-140, 22}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {68, -270}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput C5(start=0.1) annotation(
    Placement(transformation(origin = {-138, -25}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {168, -333}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput R4(start=0.1) annotation(
    Placement(transformation(origin = {-137, -68}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {239, -410}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput N2(start=0.1) annotation(
    Placement(transformation(origin = {-137, -112}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {323, -506}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput FR2_min annotation(
    Placement(transformation(origin = {84, -29}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {18, -57}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput FR2_max annotation(
    Placement(transformation(origin = {84, -65}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {20, -81}, extent = {{-10, -10}, {10, 10}})));

protected 
  parameter Real FR1_nom = 0.5;
  parameter Real FR2_nom = 0.15;
  parameter Real a = 4;
  parameter Real l = 30.5;
  parameter Real FR1_kipp_x = 13.96;
  parameter Real FR1_kipp_y = 7.25;
  parameter Real FR2_kipp_x = 31.2;
  parameter Real FR2_kipp_y = 5;
  parameter Real FR2_wackel_x = 17;
  parameter Real FR2_wackel_y = 5;

  Real t_trans, A, B, C, X1, X2, alpha1, beta, alpha;
  Real FR1_C4_min, FR1_C4_max, FR1_S4_ty_min, FR1_S4_ty_max, FR1_S4_r_min, FR1_S4_r_max;
  Real FR2_C4_min, FR2_C4_max, FR2_S4_ty_min, FR2_S4_ty_max, FR2_S4_r_min, FR2_S4_r_max;
  Real FR2_R4_r;
  
equation
  // Kinematics shared across FR1 and FR2
  t_trans = -C4;
  A = a / t_trans;
  B = a / l - 1;
  C = A^2 - B^2;
  X1 = A / C + sqrt((A^2) / (C^2) - (1 - B^2) / C);
  X2 = A / C - sqrt((A^2) / (C^2) - (1 - B^2) / C);
  alpha1 = atan(X1);
  FR1_C4_min = FR1_kipp_y * (1 - cos(alpha1)) + FR1_kipp_x * sin(alpha1);
  FR1_C4_max = -FR1_C4_min;
  FR1_S4_ty_min = FR1_C4_min;
  FR1_S4_ty_max = -FR1_S4_ty_min;
  beta = atan(S4 / 113);
  alpha = acos((a * cos(beta) + l - a) / l);
  FR1_S4_r_max = FR1_kipp_y * (1 - cos(alpha)) + FR1_kipp_x * sin(alpha);
  FR1_S4_r_min = -FR1_S4_r_max;

  FR1_min = FR1_nom + (-R1 / 2 - S2 / 2 + FR1_C4_min + FR1_S4_ty_min + FR1_S4_r_min);
  FR1_max = FR1_nom + ( R1 / 2 + S2 / 2 + FR1_C4_max + FR1_S4_ty_max + FR1_S4_r_max);

  FR2_C4_min = FR2_kipp_y * (1 - cos(alpha1)) + FR2_kipp_x * sin(alpha1);
  FR2_C4_max = -FR2_C4_min;
  FR2_S4_ty_min = FR2_C4_min;
  FR2_S4_ty_max = -FR2_S4_ty_min;
  FR2_S4_r_max = FR2_kipp_y * (1 - cos(alpha)) + FR2_kipp_x * sin(alpha);
  FR2_S4_r_min = -FR2_S4_r_max;
  FR2_R4_r = FR2_wackel_y * (1 - cos(atan(R4 / 113))) + FR2_wackel_x * sin(atan(R4 / 113));

  FR2_min = FR2_nom + (-C5 / 2 - R4 / 2 + FR2_R4_r + FR2_C4_min + FR2_S4_ty_min + FR2_S4_r_min);
  FR2_max = FR2_nom + ( C5 / 2 + R4 / 2 + FR2_R4_r + FR2_C4_max + FR2_S4_ty_max + FR2_S4_r_max);

  annotation(
    uses(Modelica(version = "4.0.0")),
    Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})),
    Icon(coordinateSystem(extent = {{-200, -200}, {200, 200}})),
    version = "");
end geom_tol;
