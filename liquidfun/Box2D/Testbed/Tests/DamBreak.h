/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#ifndef DAM_BREAK_H
#define DAM_BREAK_H

class DamBreak : public Test
{
public:

	DamBreak()
	{

		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2ChainShape shape;
			const b2Vec2 vertices[4] = {
				b2Vec2(-3, 0),
				b2Vec2(3, 0),
				b2Vec2(3, 4),
				b2Vec2(-3, 4)};
			shape.CreateLoop(vertices, 4);
			ground->CreateFixture(&shape, 0.0f);

		}

		m_particleSystem->SetRadius(0.025f);
		m_particleSystem->SetDamping(0.2f);


		m_particleSystemGravel->SetRadius(0.025f);
		m_particleSystemGravel->SetDamping(0.2f);

		{
			b2PolygonShape shape;
			shape.SetAsBox(2.9f, 1.5f, b2Vec2(0.0f, 1.6f), 0);
			b2ParticleGroupDef pd;
			pd.flags = TestMain::GetParticleParameterValue();

			pd.color.Set(100, 100, 255, 255);
			pd.shape = &shape;

			b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}

		// some weird blue oil
		{
			b2PolygonShape shape_gravel;
			shape_gravel.SetAsBox(0.2f, 0.2f, b2Vec2(0.0f, 2.6f), 0);
			b2ParticleGroupDef pd_gravel;

			pd_gravel.color.Set(0, 0, 255, 255);
			pd_gravel.flags = b2_tensileParticle; //TestMain::GetParticleParameterValue();

			pd_gravel.shape = &shape_gravel;
			// b2ParticleGroup * const group = m_particleSystemGravel->CreateParticleGroup(pd_gravel);
			b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd_gravel);
			if (pd_gravel.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 5);
			}
		}

		
		// two rows of rocks at the bottom of your tank
		for (int i = 0; i < 50; i++)

			{

				float xPos = 0 ; 
				float yPos = 0.5f;

				if (i > 25) {
					yPos += 0.5;
					xPos = -2.5+(i-25)*(0.2);
				}
				else {
					xPos = -2.5+i*(0.2);
				}

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2CircleShape shape;
				shape.m_p.Set(xPos,yPos );
				shape.m_radius = 0.1f;
				body->CreateFixture(&shape, 4.0f);
				m_particleSystem->DestroyParticlesInShape(shape,
														  body->GetTransform());
			}


		


			// a fishy

		// instead of making the animals out of rigid bodies, you could possibly make them out of rigid particle groups ?
			// it may not be possible to constrain particle objects with joints.
		{
			b2BodyDef bd1;
			bd1.type = b2_dynamicBody;
			b2Body* body1 = m_world->CreateBody(&bd1);
			b2PolygonShape shape1;
			shape1.SetAsBox(0.03f, 0.1f, b2Vec2(1, 0.5f), 0.5f);
			body1->CreateFixture(&shape1, 1.0f);
			m_particleSystem->DestroyParticlesInShape(shape1,
													  body1->GetTransform());


			b2BodyDef bd2;
			bd2.type = b2_dynamicBody;
			b2Body* body2 = m_world->CreateBody(&bd2);
			b2PolygonShape shape2;
			shape2.SetAsBox(0.03f, 0.1f, b2Vec2(1, 0.6f), 0.5f);
			body2->CreateFixture(&shape2, 1.0f);
			m_particleSystem->DestroyParticlesInShape(shape2,
													  body2->GetTransform());




			// some notes about serializing joints. https://stackoverflow.com/questions/6950587/how-to-extract-create-a-jointdef-based-off-a-b2joint-instantiation

		    b2DistanceJointDef jointDef1;
			jointDef1.bodyA = body1;
			jointDef1.bodyB = body2;
			jointDef1.localAnchorA =  b2Vec2(1.0f, 0.65f); //SetZero();
			jointDef1.localAnchorB =  b2Vec2(1.0f, 0.65f);//SetZero();
			jointDef1.frequencyHz = 100.0f;
			jointDef1.dampingRatio = 0.0f;
			jointDef1.length = 0.0f;
			m_world->CreateJoint(&jointDef1);

			b2RevoluteJointDef jointDef2;
			jointDef2.bodyA = body1;
			jointDef2.bodyB = body2;
			jointDef2.localAnchorA =  b2Vec2(1.0f, 0.65f); //SetZero();
			jointDef2.localAnchorB =  b2Vec2(1.0f, 0.65f);//SetZero();
			// jointDef1.frequencyHz = 100.0f;
			// jointDef1.dampingRatio = 0.0f;
			// jointDef1.length = 0.0f;
			jointDef2.lowerAngle = -1.0f;
			jointDef2.upperAngle = 1.0f;

			jointDef2.enableMotor = true;
            jointDef2.maxMotorTorque = 100.0f;
            jointDef2.motorSpeed = 10.0f;
       

            int balrge = 909;
            jointDef2.userData = &balrge;

			m_world->CreateJoint(&jointDef2);




		}




	}


	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new DamBreak;
	}
};

#endif
