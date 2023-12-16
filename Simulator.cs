//Simulator
//=========================================================
//Simulator.cs : Defines the base class for creating simulations
//=========================================================
using System;

public class Simulator
{
    double spring_constant;      //a new variable is needed to take the spring into account
    double pendulum_mass;   //mass has to be taken into account as it no longer cancels out in our RHS equation
     double TA;      // X,Y,Z original length of spring
    protected int n;    
    protected double[] x;   //array of x,y,z states
    
    protected double[] xA;  //intermediate states for x,y, and z from RK2-4
    protected double[] xB;
    protected double[] xC;      
    
    protected double[][]x_prime;    //xyz arrays that holds values of RHS
                                    //known as array of array of slopes
    protected double g;     //gravitational field strength
    private Action<double[],double[], double, double,double,double> rhsFunc;
    //-----------------------------
    //Constructor
    //-----------------------------
    public Simulator(int nn)
    {
        g = 9.81;
        n = nn;
        x = new double[n];
        xA = new double[n];
        xB = new double[n];
        xC = new double[n];
        x_prime = new double [4][];
        x_prime[0] = new double[n];
        x_prime[1] = new double[n];
        x_prime[2] = new double[n];
        x_prime[3] = new double[n];

        rhsFunc = nothing;
    }
    //--------------------------------------
    // StepEuler: excutes ODE a single time
    //--------------------------------------
    public void StepEuler(double time, double dTime)
    {
         int i;

        rhsFunc(x, x_prime[0], spring_constant, time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + x_prime[0][i] * dTime;
        }
    }
     //------------------------------------------------
    // StepRK2:  completes one RK2 step in solving
    //          equations of motion
    //------------------------------------------------
    public void StepRK2(double time, double dTime)
    {
        int i;
        rhsFunc(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xA[i] = x[i] + x_prime[0][i] * dTime;
        }
        rhsFunc(x, x_prime[1], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + 0.5 * (x_prime[0][i] + x_prime[1][i]) * dTime;
        }
        
    }


    //------------------------------------------------  
    // StepRK4:  completes one RK4 step in solving
    //          equations of motion
    //------------------------------------------------
    public void StepRK4(double time, double dTime)
    {
        int i;
        rhsFunc(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xA[i] = x[i] + 0.5 * x_prime[0][i] * dTime;
        }
        rhsFunc(x, x_prime[1], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xB[i] = x[i] + 0.5 * x_prime[1][i] * dTime;
        }
        rhsFunc(x, x_prime[2], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xC[i] = x[i] + x_prime[2][i] * dTime;
        }
        rhsFunc(x, x_prime[3], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + 1.0/6.0 * (x_prime[0][i] + 2 * x_prime[1][i] + 2 * x_prime[2][i] + x_prime[3][i]) * dTime;
        }
    }
   protected void SetRHSFunc(Action<double[],double[], double, double,double,double> rhs)
   {
        rhsFunc = rhs;
   }
   private void nothing(double[] x,double[] x_prime, double spring_constant,double time,double TA,double pendulum_mass)
   {

   }
   
}
    




