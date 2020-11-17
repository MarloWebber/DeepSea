/*
*/

#include "../Framework/DeepSea.h"

#ifndef OLYMPICPOOL_H
#define OLYMPICPOOL_H

class OlympicPool : public Test
{
public:

	OlympicPool()
	{

		{

			m_world->SetGravity(b2Vec2(0.0f,0.0f));

			b2BodyDef bd;
			uDataWrap * p_dataWrapper = new uDataWrap(nullptr, TYPE_DEFAULT);
			bd.userData = (void*)p_dataWrapper;
			b2Body* ground = m_world->CreateBody(&bd);

			b2ChainShape shape;

			// an octagon
			const b2Vec2 vertices[8] = {
				b2Vec2(25, 5),
				b2Vec2(-25, 5),
				b2Vec2(-25, -5),
				b2Vec2(25, -5)};
				// b2Vec2(-10, 20),
				// b2Vec2(-20, 10),
				// b2Vec2(-20, -10),
				// b2Vec2(-10, -20)};
			shape.CreateLoop(vertices, 4);
			ground->CreateFixture(&shape, 0.0f);

		}

		m_particleSystem->SetRadius(0.2f);
		m_particleSystem->SetDamping(0.2f);

		// {
		// 	//  water A
		// 	b2PolygonShape shape;
		// 	shape.SetAsBox(25.0f, 5.0f, b2Vec2(0.0f, 0.0f), 0);
		// 	b2ParticleGroupDef pd;
		// 	pd.flags = TestMain::GetParticleParameterValue();

		// 	// royal azure
		// 	pd.color.Set(0, 0x38, 0xa8, 255);
		// 	pd.shape = &shape;

		// 	b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd);
		// 	if (pd.flags & b2_colorMixingParticle)
		// 	{
		// 		ColorParticleGroup(group, 0);
		// 	}
		// }


		// {
		// 	//  water B
		// 	b2PolygonShape shape;
		// 	shape.SetAsBox(7.5f, 15.0f, b2Vec2(-7.5f, 0.0f), 0);
		// 	b2ParticleGroupDef pd;
		// 	pd.flags = TestMain::GetParticleParameterValue();

		// 	// pastel gold
		// 	pd.color.Set(255, 200, 0, 255);
		// 	pd.shape = &shape;

		// 	b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd);
		// 	if (pd.flags & b2_colorMixingParticle)
		// 	{
		// 		ColorParticleGroup(group, 0);
		// 	}
		// }

	}


	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new OlympicPool;
	}
};

#endif
