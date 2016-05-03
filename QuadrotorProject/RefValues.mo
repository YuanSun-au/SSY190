within QuadrotorProject;

model RefValues "Generate reference values"
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealOutput ref[nRef] "Connector of Real reference signals as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
equation
  //der(ref[1]) = 2 - ref[1];
  ref[1]=0;
  ref[2]=0;
  ref[3]=0;
  ref[4]=0;
  ref[5]=0;
  ref[6]=0;
  ref[7]=0;
  ref[8]=0;
  ref[9]=0;
  //ref[10]=1;
  der(ref[10]) = 1 - ref[10];
  ref[11]=0;
  ref[12]=0;
  //der(ref[12]) = 1 - ref[12];
  annotation(Documentation(info = "<html><h1 class=\"heading\">REFERENCE VALUE GENERATION</h1><p>Use this block to generate the reference values for the system.</p></html>"));
end RefValues;