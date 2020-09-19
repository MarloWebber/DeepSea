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

// #include "DeepSea.h"

class DamBreak : public Test
{
public:

	DamBreak()
	{

		{

			m_world->SetGravity(b2Vec2(0.0f,0.0f));

			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2ChainShape shape;
			const b2Vec2 vertices[4] = {
				b2Vec2(-30, -40),
				b2Vec2(30, -40),
				b2Vec2(30, 40),
				b2Vec2(-30, 40)};
			shape.CreateLoop(vertices, 4);
			ground->CreateFixture(&shape, 0.0f);

		}

		m_particleSystem->SetRadius(0.025f);
		m_particleSystem->SetDamping(0.2f);


		// m_particleSystemGravel->SetRadius(0.025f);
		// m_particleSystemGravel->SetDamping(0.2f);

		{
			// the water
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 0.1f, b2Vec2(0.0f, 100.0f), 0);
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
			shape_gravel.SetAsBox(0.1f, 0.1f, b2Vec2(0.0f, 100.0f), 0);
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
		// for (int i = 0; i < 50; i++)

		// 	{

		// 		float xPos = 0 ; 
		// 		float yPos = 0.5f;

		// 		if (i > 25) {
		// 			yPos += 0.5;
		// 			xPos = -2.5+(i-25)*(0.2);
		// 		}
		// 		else {
		// 			xPos = -2.5+i*(0.2);
		// 		}

		// 		b2BodyDef bd;
		// 		bd.type = b2_dynamicBody;
		// 		b2Body* body = m_world->CreateBody(&bd);
		// 		b2CircleShape shape;
		// 		shape.m_p.Set(xPos,yPos );
		// 		shape.m_radius = 0.1f;
		// 		body->CreateFixture(&shape, 4.0f);
		// 		m_particleSystem->DestroyParticlesInShape(shape,
		// 												  body->GetTransform());
		// 	}



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
