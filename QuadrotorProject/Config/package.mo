within QuadrotorProject;

package Config
  //Import libraries
  import Modelica.SIunits;
  // Define the number of inputs and outputs from each block
  final constant Integer nRef = 12 "Number of reference signals";
  final constant Integer nCtrl = 4 "Number of control signals";
  final constant Integer nOut = 12 "Number of plant output signals";
  final constant Integer nSens = 12 "Number of sensor signals";
  // Quadrotor specific variables
  final constant SIunits.Acceleration g = 9.81 "Gravitational acceleration";
  final constant SIunits.Mass m = 0.027 "Mass of the quadrotor";
  final constant SIunits.Length L = 0.1 "Length of quadcopter arms (from center to center of the rotors diagonally)";
  final constant SIunits.Inertia Ix = 1.395e-5 "Moment of inertia, x-axis";
  final constant SIunits.Inertia Iy = 1.436e-5 "Moment of inertia, y-axis";
  final constant SIunits.Inertia Iz = 2.173e-5 "Moment of inertia, z-axis";
  // DOCUMENTATION
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONFIGURATION OF THE QUADROTOR PROJECT</h1><p>Here you can store necessary configuration parameters that is used in the model</p></html>"));
end Config;
