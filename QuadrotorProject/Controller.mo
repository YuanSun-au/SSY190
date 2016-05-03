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
  K=[3.59124352545129e-13	1.03696489842753e-10	-3.46356197333444e-14	1.75850402595064e-18	4.74422184879216e-16	-4.41554732595435e-19	-8.57951516382323e-12	8.29288040077722e-14	3.18916336891811	-3.17128437786726e-12	5.29756052656468e-14	3.16227766016840;
  17.1696173355402	3.50379340135710e-10	2.33623969816423e-14	3.16235340092211	1.62383174481943e-15	-5.70308207594074e-17	-5.95771918262690e-11	4.59013473299154	3.40355617925930e-15	-3.36288359025866e-11	3.16227766015514	5.46282636011298e-14;
  3.72444480694116e-10	17.1696231740994	-4.35615351311130e-13	1.57746886074033e-15	3.16235562699351	-7.89660018608972e-18	-4.59013514308437	9.09315708831156e-11	8.92019428394068e-13	-3.16227766021366	1.08846037465233e-10	1.04807057845921e-10;
  -7.69285166047532e-12	-1.01144519327291e-12	3.16227766016783	-3.66120547443043e-17	-5.21836993429583e-18	3.16229939009372	2.73844161834560e-13	-2.53019756937525e-12	-5.48641407274586e-16	1.82452756485787e-13	-1.79274189595366e-12	-1.67253116475509e-14];
equation
  connect(ref[1], add.u1) annotation(Line(points = {{-120, 40}, {-80, 40}, {-80, 6}, {-62, 6}}, color = {0, 0, 127}));
  connect(sens[1], add.u2) annotation(Line(points = {{-120, -40}, {-80, -40}, {-80, -6}, {-62, -6}}, color = {0, 0, 127}));
  connect(add.y, PID.u) annotation(Line(points = {{-39, 0}, {100, 0}}, color = {0, 0, 127}));
  //ctrl = {PID.y/4, PID.y/4, PID.y/4, PID.y/4};
  //ctrl={65,65,65,65};
  ctrl = -K*sens;
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT IN MODELICA PID</h1><p>Uses only default modelica blocks to model the controller</p></html>"));
end Controller;
