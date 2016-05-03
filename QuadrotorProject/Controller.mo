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

  //Feedback gain vector
//  Real[4,12] K=[ -0.0000 ,   0.0000  , -0.0000 ,  -0.0000   , 0.0000 ,  -0.0000  , -0.0000 ,  -0.0000 ,   3.2465  , -0.0000 ,  -0.0000  , 10.0000;
//   17.1696 ,  -0.0000  , -0.0000  ,  3.1624  , -0.0000 ,  -0.0000    ,0.0000  ,  4.5901 ,  -0.0000 ,   0.0000   , 3.1623 ,  -0.0000;
//   -0.0000 ,  17.1696 ,   0.0000 ,  -0.0000  ,  3.1624  ,  0.0000  , -4.5901 ,  -0.0000  ,  0.0000  , -3.1623  , -0.0000  ,  0.0000;
//   -0.0000  ,  0.0000   , 3.1623 ,  -0.0000 ,   0.0000  ,  3.1623  , -0.0000  , -0.0000  , -0.0000  ,  0.0000  , -0.0000  ,  0.0000];

Real[4,12] K=[    0.00000,       0.00000,       0.00000,       0.00000,       0.00000,      -0.00000,      -0.00000,       0.00000,      31.62548,      -0.00000,      -0.00000,       3.16228;
     146.72090,      -0.00000,       0.00000,      31.62284,      -0.00000,      -0.00000,       0.00000,      33.08461,       0.00000,       0.00000,       3.16228,       0.00000;
      -0.00000,     146.72090,      -0.00000,      -0.00000,      31.62284,      -0.00000,     -33.08461,      -0.00000,       0.00000,      -3.16228,      -0.00000,       0.00000;
      -0.00000,      -0.00000,      31.62278,      -0.00000,      -0.00000,      31.62280,       0.00000,      -0.00000,      -0.00000,      -0.00000,       0.00000,      -0.00000];

equation
  connect(ref[1], add.u1) annotation(Line(points = {{-120, 40}, {-80, 40}, {-80, 6}, {-62, 6}}, color = {0, 0, 127}));
  connect(sens[1], add.u2) annotation(Line(points = {{-120, -40}, {-80, -40}, {-80, -6}, {-62, -6}}, color = {0, 0, 127}));
  connect(add.y, PID.u) annotation(Line(points = {{-39, 0}, {100, 0}}, color = {0, 0, 127}));
  //ctrl = {PID.y/4, PID.y/4, PID.y/4, PID.y/4};
  //ctrl={65,65,65,65};
  ctrl = K*(sens-ref);
  //ctrl = {0.26,0,0.001,0}
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT IN MODELICA PID</h1><p>Uses only default modelica blocks to model the controller</p></html>"));
end Controller;
