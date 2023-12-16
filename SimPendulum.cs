using System;

public class SimPendulum
{
    private double L;   // pendulum length this is not needed as we are using a spring
    private double g;   //gravity 
    double spring_constant;     //a new variable is needed to take the spring into account
    double pendulum_mass;   //mass has to be taken into account as it no longer cancels out in our RHS equation
    int n;  //number of ODEs to run, since its 3d we need 6
    double [] x;    //array of states for x position
    double [][] x_prime;  // array of array of slopes
    double [] xA;  //additional intermediate state for RK2 and RK4
    double [] xB;  //additional intermediate state for RK4
    double [] xC;  //additional intermediate state for RK4
    double TA;      // X,Y,Z original length of spring

    //---------------------------------------
    // Constructor
    //---------------------------------------
    public SimPendulum()
    {
        L = 0.9;
        g = 9.81;
        TA = 20.0;      //original unstreched spring length
        
        n = 6;          // n is equal to 6 for our case
        x = new double[n];
        xA = new double[n];
        xB = new double[n];
        xC = new double[n];
        x_prime = new double[2][];
        x_prime[0] = new double[n];
        x_prime[1] = new double[n];

       // initial default condition for pendulum
        x[0] = 1.0;     
        x[1] = 1.0;     

       x[2] = 1.0;               
        x[3] = 1.0;          
        
        x[4] = 1.0;            
        x[5] = 1.0;          
        spring_constant = 0.5;
        pendulum_mass = 1.0;
    } 
    //-----------------------------------------------
    // StepEuler: completes one Euler step in solving
    //            equations of motion
    //-----------------------------------------------
    public void StepEuler(double time, double dt)
    {
        int i;

        Right_Hand_Side_Function(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + x_prime[0][i] * dt;
        }
    }
    //------------------------------------------------
    // StepRK2:  completes one RK2 step in solving
    //          equations of motion
    //------------------------------------------------
    public void StepRK2(double time, double dt)
    {
        int i;
         Right_Hand_Side_Function(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xA[i] = x[i] + x_prime[0][i] * dt;
        }
         Right_Hand_Side_Function(x, x_prime[0], spring_constant,time+dt, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + 0.5 * (x_prime[0][i]+x_prime[1][i]) * dt;
        }
        
    }


    //------------------------------------------------  
    // StepRK4:  completes one RK4 step in solving
    //          equations of motion
    //------------------------------------------------
    public void StepRK4(double time, double dt)
    {
        int i;
         Right_Hand_Side_Function(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xA[i] = x[i] + 0.5 * x_prime[0][i] * dt;
        }
        Right_Hand_Side_Function(x, x_prime[0], spring_constant,time+dt, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xB[i] = x[i] + 0.5 * x_prime[1][i] * dt;
        }
        Right_Hand_Side_Function(x, x_prime[0], spring_constant,time+dt, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            xC[i] = x[i] + x_prime[2][i] * dt;
        }
        Right_Hand_Side_Function(x, x_prime[0], spring_constant,time, TA, pendulum_mass);
        for(i=0; i<n; ++i)
        {
            x[i] = x[i] + 1.0/6.0 * (x_prime[0][i] + 2 * x_prime[1][i] + 2 * x_prime[2][i] + x_prime[3][i]) * dt;
        }
    }
    //---------------------------------------
    // RHSFuncPendulum
    //---------------------------------------
    private void Right_Hand_Side_Function(double[] xx,double[] xx_prime,double spring_constant,double t,double TA,double pendulum_mass)
    {
       // x_prime[0] = x[1];
        //x_prime[1] = -spring_constant*(Math.Sqrt(Math.Pow(x[0],2) + Math.Pow(x[2],2) + Math.Pow(x[0],4)) - TA)/pendulum_mass - g;
        xx_prime[0] = x[1];
        xx_prime[1] = -spring_constant*(Math.Sqrt(Math.Pow(x[0],2) + Math.Pow(x[2],2) + Math.Pow(x[0],4)) - TA)/pendulum_mass - g;
    }

    

    public void TestFunc()
    {
        Console.WriteLine("inside testfunc");
    }
}