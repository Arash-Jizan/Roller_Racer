//===================================
// PendSim.cs : Defines derived class
//===================================



using System;
using System.Reflection.Emit;
using Godot;

public class PendSim : Simulator
{
  double spring_constant;
  double pendulum_mass;
  double TA;
  double kinetic_energy1;
  double kinetic_energy2;
  double kinetic_energy3;
  double kinetic_energy4;
    
    public PendSim() : base(6)
    {
       TA = 20.0;
        spring_constant = 0.5;
       pendulum_mass = 1.0;
       // L= 1;

        x[0] = 1.0;
        x[1] = 1.0;
        x[2] = 1.0;      
        x[3] = 1.0;   
        x[4] = 1.0;   
        x[5] = 1.0;   
        SetRHSFunc(Right_Hand_Side_Function);     



        

    }
    private void Right_Hand_Side_Function(double[] xx,double[] xx_prime,double spring_constant,double time,double TA,double pendulum_mass)
    {
        xx_prime[0] = x[1];
        xx_prime[1] = -spring_constant*(Math.Sqrt(Math.Pow(x[0],2) + Math.Pow(x[2],2) + Math.Pow(x[4],4)) - TA)/pendulum_mass - g;    
        

    }
    // function to find kinetic energy it is 
    // an additional getter and setter 
    public double Velocity_for_ke()
    {
      
      
    kinetic_energy1= 1.0/2.0*pendulum_mass*65;
    return kinetic_energy1;
        
   }
    //-------------------------------
    //GETTERS
    //-------------------------------
     public double dimension_x
    {
        get
        {
            return(x[0]);
        }
        
        set
        {
            x[0] = value;
        }
    }
    
    public double dimension_y
     {
        get
        {
            return(x[2]);
        }
        
        set
        {
            x[2] = value;
        }
    }
    public double dimension_z
    {
        get
        {
            return(x[4]);
        }
        
        set
        {
            x[4] = value;
        }
    }
    
}