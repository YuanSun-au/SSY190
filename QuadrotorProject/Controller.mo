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
  Real[4, 12] K = [-5.37190096920152e-11, 2.46728504794558e-08, 1.31058286944190e-11, -7.45115722643570e-18, 3.80174153975789e-15, 6.27967868333415e-17, -2.11597239083005e-09, -1.62301622564355e-11, 100.026996355984, -9.65934806008483e-10, -1.25416247534296e-11, 100.000000000000; 542.944892636668, -1.00891282857223e-06, -1.93797753619591e-11, 100.000075740784, -1.44285418980593e-13, 1.14825230827246e-15, 1.92682302101353e-07, 145.152378472878, -1.44215946318110e-14, 1.14564786972796e-07, 100.000000479641, 1.10124285071335e-11; -1.00676138894224e-06, 542.944896241251, -3.91572234953395e-12, -1.40165849218612e-13, 100.000077966857, -1.26302071899862e-16, -145.152378447730, -2.11389069937653e-07, 7.14812127948907e-12, -99.9999999954402, -1.63520967683317e-07, 2.45444513517031e-08; 4.64489141662048e-09, -6.18027549732188e-10, 100.000000000025, 7.37143106323094e-16, -8.34651519780033e-17, 100.000021729998, 2.55803090632592e-10, 1.40575390010133e-09, 7.80263803267473e-14, 2.00216639298309e-10, 1.02100161957579e-09, 7.92817665953144e-11];
equation
  connect(ref[1], add.u1) annotation(Line(points = {{-120, 40}, {-80, 40}, {-80, 6}, {-62, 6}}, color = {0, 0, 127}));
  connect(sens[1], add.u2) annotation(Line(points = {{-120, -40}, {-80, -40}, {-80, -6}, {-62, -6}}, color = {0, 0, 127}));
  connect(add.y, PID.u) annotation(Line(points = {{-39, 0}, {100, 0}}, color = {0, 0, 127}));
//ctrl = {PID.y};
//ctrl={65,65,65,65};
  ctrl = K * sens;
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT IN MODELICA PID</h1><p>Uses only default modelica blocks to model the controller</p></html>"));
end Controller;