/*
*/
#ifndef SUNPOOL_H
#define SUNPOOL_H

class SunPool : public Test
{
public:

	SunPool()
	{

		{

			m_world->SetGravity(b2Vec2(0.0f,0.0f));

			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2ChainShape shape;

			// an octagon
			const b2Vec2 vertices[8] = {
				b2Vec2(10, -20),
				b2Vec2(20, -10),
				b2Vec2(20, 10),
				b2Vec2(10, 20),
				b2Vec2(-10, 20),
				b2Vec2(-20, 10),
				b2Vec2(-20, -10),
				b2Vec2(-10, -20)};
			shape.CreateLoop(vertices, 8);
			ground->CreateFixture(&shape, 0.0f);

		}

		m_particleSystem->SetRadius(0.25f);
		m_particleSystem->SetDamping(0.2f);

		{
			// the water
			b2PolygonShape shape;
			shape.SetAsBox(15.0f, 15.0f, b2Vec2(0.0f, 0.0f), 0);
			b2ParticleGroupDef pd;
			pd.flags = TestMain::GetParticleParameterValue();

			// pastel red
			pd.color.Set(225, 36, 35, 255);
			pd.shape = &shape;

			b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}
	}


	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new SunPool;
	}
};

#endif
