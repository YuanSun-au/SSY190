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
  Real roll,pitch,yaw; // Rotation states
  Real p,q,r; // Angular velocity

  Real x,y,z; //linear pos
  Real u,v,w; //linear speed


//Rotation matrix
  Real R[3,3]; //R=Rx(roll)*Ry(pitch)*Rz(yaw);
  Real T[3,3];

// Control signals
  Real ft = ctrl[1];
  Real Tx = ctrl[2];
  Real Ty = ctrl[3];
  Real Tz = ctrl[4];
equation
// ADD YOUR EQUATION BELOW
// Rotation matrix xyz
//  R=[cos(pitch)*cos(yaw) , - cos(pitch)*sin(yaw) , sin(pitch) ;
//cos(roll)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch) , cos(roll)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw) , - cos(pitch)*sin(roll) ;
//sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch) , cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw) , cos(roll)*cos(pitch) ];
// Rotation matrix zyz euler
  R=[cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw) -cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw)+ sin(roll)*sin(pitch);
     cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(pitch), cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
     -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch) ];

  [der(x);der(y);der(z)]=R*[u;v;w];


  // Rotation of angular velocity
  T=[1,sin(roll)*tan(pitch),cos(roll)*tan(pitch);
   0,cos(roll),-sin(roll);
    0, sin(roll)/cos(pitch),cos(roll)/cos(pitch)];

  [der(roll); der(pitch); der(yaw)] = T*[p;q;r];

  // Dynamic equations
 -m*g*sin(pitch) = m*(der(u) + q*w - r*v);
 m*g*cos(pitch)*sin(roll) = m*(der(v) - p*w + r*u);
 m*g*cos(pitch)*cos(roll) -ft = m*(der(w) + p*v -q*u);
 Tx = der(p)*Ix - q*r*Iy + q*r*Iz;
 Ty = der(q)*Iy + p*r*Ix - p*r*Iz;
 Tz = der(r)*Iz - p*q*Ix + p*q*Iy;


  // all states as output
  plantOut={roll,pitch,yaw,p,q,r,u,v,w,x,y,z};
  //
  // NOTHING MORE TO CHANGE BELOW
  annotation(Documentation(info = "<html><h1 class=\"heading\">QUADROTOR BLOCK</h1><p></p><p>Implement your own plant model equation and update the force and torque that affects the Body of the quadrotor.</p></html>"));
end Quadrotor;
