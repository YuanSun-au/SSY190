within QuadrotorProject;

model Controller
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealInput ref[nRef] "Connector of Real reference signals as input" annotation(Placement(transformation(extent = {{-140, 20}, {-100, 60}}, rotation = 0)));
  RealInput sens[nSens] "Connector of Real sensor signals as input" annotation(Placement(transformation(extent = {{-140, -60}, {-100, -20}}, rotation = 0)));
  RealOutput ctrl[nCtrl] "Connector of Real control signals as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PID PID(Ti = 0.5, Td = 0.04, k = 1e2) annotation(Placement(visible = true, transformation(origin = {-16, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k2 = -1) annotation(Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ref[1], add.u1) annotation(Line(points = {{-120, 40}, {-80, 40}, {-80, 6}, {-62, 6}}, color = {0, 0, 127}));
  connect(sens[1], add.u2) annotation(Line(points = {{-120, -40}, {-80, -40}, {-80, -6}, {-62, -6}}, color = {0, 0, 127}));
  connect(add.y, PID.u) annotation(Line(points = {{-39, 0}, {100, 0}}, color = {0, 0, 127}));
  //ctrl = {PID.y};
  ctrl={65,65,65,65};
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT IN MODELICA PID</h1><p>Uses only default modelica blocks to model the controller</p></html>"));
end Controller;