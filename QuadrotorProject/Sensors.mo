within QuadrotorProject;

model Sensors
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealInput plantOut[nOut] "Connector of Real plant output as input" annotation(Placement(transformation(extent = {{140, -20}, {100, 20}}, rotation = 0)));
  RealOutput sens[nSens] "Connector of Real sensor data as output" annotation(Placement(transformation(extent = {{-100, -10}, {-120, 10}}, rotation = 0)));
equation
  sens = {plantOut};
  annotation(Documentation(info = "<html><h1 class=\"heading\">SENSOR EMULATION</h1><p>This block should translate the plant output to sensor data. Consider to implement additional sensors, sample rate, noise and disturbances.</p></html>"));
end Sensors;
