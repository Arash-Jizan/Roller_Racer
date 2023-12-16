using Godot;
using System;

public partial class simulator_begin_scene : Node3D
{
	MeshInstance3D anchor;
	MeshInstance3D Ball;
	SpringModel spring;
	Label keLabel;
	double kinetic_energy_x;
	double kinetic_energy_y;
	double kinetic_energy_z;
	PendSim pend;

	double xA, yA, zA;		//cordinates of the anchor where pend swings from
	double xB, yB, zB;		//cordinates of the ball where pend swings from

	float length;		//length of the string
	float length0; 		//natural pendulum length
	double angle;		// angle of the pendulum
	double pendulum_mass;
	double KeValue;
	double angleInit;

	Vector3 endA;		//point where the anchor and spring meet
	Vector3 endB;		// point where the spring and pend bob meet
	double time;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		
		keLabel = GetNode<Label>("KElabel");
		xA = 0.0; yA = 3.1; zA = 0.0;

		xB = 1.0; yB = 1.0; zB = 1.0;//basic starting position to inject into getters


		anchor = GetNode<MeshInstance3D>("Anchor");
		Ball = GetNode<MeshInstance3D>("Ball");
		spring = GetNode<SpringModel>("SpringModel");
		length0 = length = 2.0f;
		//angle = Mathf.DegToRad(60.0);
		


		anchor.Position = new Vector3((float)xA, (float)yA, (float)zA);
		
		pend = new PendSim();
		
		endA = new Vector3((float)xA, (float)yA, (float)zA);
		anchor.Position = endA;
		
		spring.GenMesh(0.15f, 0.045f, length, 6.0f, 62);


		
		angleInit = Mathf.DegToRad(60.0);
		float angleF = (float)angleInit;

		
		pend.dimension_x = xB;
		pend.dimension_y = yB;
		pend.dimension_z = zB;
		kinetic_energy_x = xB;
		kinetic_energy_y = yB;
		kinetic_energy_z = zB;

		endB.X = endA.X + length*Mathf.Sin(angleF);
		endB.Y = endA.Y - length*Mathf.Cos(angleF);
		endB.Z = endA.Z;
		PlacePendulum(endB);
		time = 0.0;
		
		
	}
	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		float angleF = (float)Math.Sin(3.0 * time);
		length = length0 + 0.5f*(float)Math.Cos(5.0f * time);
		endB.X = endA.X + length*Mathf.Sin(angleF);
		endB.Y = endA.Y - length*Mathf.Cos(angleF);
		endB.Z = endA.Z;
		PlacePendulum(endB);
		time = time + delta;


		xB = pend.dimension_x;
		keLabel.Text = "Kinetic energy: " + xB.ToString("0.00");
		
		
		
	}
	public override void _PhysicsProcess(double delta)
	{
		base._PhysicsProcess(delta);
		pend.StepRK4(time, delta);
		
	}
	private void PlacePendulum(Vector3 endB)
	{
		spring.PlaceEndPoints(endA, endB);
		Ball.Position = endB;
	}
}
