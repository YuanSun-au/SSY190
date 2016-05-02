model Quadrotor
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  // Inputs should be all controll signals from your controller
  RealInput ctrl[nCtrl] "Connector of Real control signals as input" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
  // Output should be all states and output that you want to be able to use in the Sensor block.
  RealOutput plantOut[nOut] "Connector of Real plant outputs as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  // Quadrotor components
  Real roll, pitch, yaw;
  // Rotation states
  Real rpy[3, 1] = [roll; pitch; yaw];
  //Real x,y,z; // Location states
  Real v1, v2, v3;
  Real v[3, 1] = [v1; v2; v3];
  Real x, y, z;
  Real T;
  Real R[3, 3];
  //R=Rx(roll)*Ry(pitch)*Rz(yaw);
  parameter Real b = 7e-2, k = 1e-3;
  parameter Real d = L;
  parameter Real A[4, 4] = [-b, -b, -b, -b; 0, -d * b, 0, d * b; d * b, 0, -d * b, 0; k, -k, k, -k];
  Real om1, om2, om3;
  Real omega[3, 1];
  // omeaga as in angular velocity
  Real Tx, Ty, Tz;
  Real w1 = ctrl[1];
  // Rotor speed,already squared
  Real w2 = ctrl[2];
  Real w3 = ctrl[3];
  Real w4 = ctrl[4];
  parameter Real J[3, 3] = [Ix, 0, 0; 0, Iy, 0; 0, 0, Iz];
equation
// ADD YOUR EQUATION BELOW
// Rotation matrix xyz
//  R=[cos(pitch)*cos(yaw) , - cos(pitch)*sin(yaw) , sin(pitch) ;
//cos(roll)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch) , cos(roll)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw) , - cos(pitch)*sin(roll) ;
//sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch) , cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw) , cos(roll)*cos(pitch) ];
// Rotation matrix zyz euler
  R = [cos(roll) * cos(yaw) - cos(pitch) * sin(roll) * sin(yaw), (-cos(yaw) * sin(roll)) - cos(roll) * cos(pitch) * sin(yaw), sin(pitch) * sin(yaw); cos(pitch) * cos(yaw) * sin(roll) + cos(roll) * sin(yaw), cos(roll) * cos(pitch) * cos(yaw) - sin(roll) * sin(yaw), -cos(yaw) * sin(pitch); sin(roll) * sin(pitch), cos(roll) * sin(pitch), cos(pitch)];
  v = [der(x); der(y); der(z)];
// Speed vector
//v=[v1;v2;v3];
  m * der(v) = [0; 0; m * g] + R * [0; 0; T];
// Newton equation
  [T; Tx; Ty; Tz] = A * [w1; w2; w3; w4];
  omega = [om1; om2; om3];
  omega = [1, 0, -sin(pitch); 0, cos(roll), cos(pitch) * sin(roll); 0, -sin(roll), cos(pitch) * cos(roll)] * [der(roll); der(pitch); der(yaw)];
//omega=[der(roll);der(pitch);der(yaw)];
//  J*der(omega) = cross(-omega,J*omega) + [Tx;Ty;Tz];
  J * der(omega) = [Tx; Ty; Tz] - [(Iy - Iz) * omega[2] * omega[3]; (Iz - Ix) * omega[1] * omega[3]; (Ix - Iy) * omega[1] * omega[2]];
//
// SORT OUT THE PLANT OUTPUT HERE
// control input = force applied vertically by the rotors
// plant output = {altitude, vertical speed}
  plantOut = {pitch, yaw, roll, om2, om3, om1, v1, v2, v3, x, y, z};
//
//
// NOTHING MORE TO CHANGE BELOW
  annotation(Documentation(info = "<html><h1 class=\"heading\">QUADROTOR BLOCK</h1><p></p><p>Implement your own plant model equation and update the force and torque that affects the Body of the quadrotor.</p></html>"));
end Quadrotor;