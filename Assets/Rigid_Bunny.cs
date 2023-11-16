using UnityEngine;
using System.Collections;
using JetBrains.Annotations;
using UnityEngine.UIElements;
using System;
using Unity.VisualScripting;
using TMPro;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision
	float friction      = 0.2f;                 

	Vector3 G = new Vector3(0, -9.8f, 0);       // gravity

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		// Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
        // Get every vertex of the object (local coordinates)
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        // Get the global coordinate rotation matrix R and the translation vector T
        Vector3 T = transform.position;
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

		Vector3 collisionVertex = new(0, 0, 0);   // collision vertex
		int collisionNum = 0;                     // number of collistion vertex

        for (int i = 0; i < vertices.Length; i++)
        {
			// Calculate the distance d from each vertex to the surface
			Vector3 ri = vertices[i];
            Vector3 Rri = R * ri;
            Vector3 xi = T + Rri;
            if (Vector3.Dot(xi - P, N) < 0.0f)
			{
                // Determine whether the object is still moving into the wall
                Vector3 vi = v + Vector3.Cross(w, Rri);
                if (Vector3.Dot(vi, N) < 0.0f)
				{
                    collisionVertex += ri;
					collisionNum++;
				}
            }
        }

		if (collisionNum == 0) return;

        // Get the global coordinate inertia tensor I
        Matrix4x4 I = R * I_ref * R.transpose;
		Matrix4x4 I_inverse = I.inverse;

        // Calculate the virtual collision point
        Vector3 r_collision = collisionVertex / collisionNum;
        Vector3 Rr_collision = R * r_collision;				
        Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);

        // Compute the wanted vi_new
        Vector3 vN = (Vector3.Dot(v_collision, N)) * N;
        Vector3 vT = v_collision - vN;
        float a = 1.0f - friction * (1.0f + restitution) * vN.magnitude / vT.magnitude;
        if (a < 0.0f) { a = 0.0f; }
        Vector3 v_new = -restitution * vN + a * vT;

        // Compute the impulse j
        Matrix4x4 K = Get_Cross_Matrix(Rr_collision) * I_inverse * Get_Cross_Matrix(Rr_collision);
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                K[r, c] = -K[r, c];
                if (r == c) { K[r, c] += 1 / mass; }
            }
        }
        Vector3 j = K.inverse * (v_new - v_collision);

		// Update v and w
		v += 1.0f / mass * j;
		w += I_inverse.MultiplyVector(Vector3.Cross(Rr_collision, j));
    }

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if(launched)
		{
            // Part I: Update velocities
            v += G * dt;
            v *= linear_decay;
            w *= angular_decay;

            // Part II: Collision Impulse
            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;
            x += dt * v;

            //Update angular status
            Quaternion q = transform.rotation;
			Quaternion tmp = new Quaternion(0.5f * dt * w[0], 0.5f * dt * w[1], 0.5f * dt * w[2], 0) * q;
            for (int i = 0; i < 4; i++)
            {
                q[i] += tmp[i];
            }

            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
        }
    }
}
