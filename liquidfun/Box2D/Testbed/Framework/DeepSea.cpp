#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

int currentNumberOfFood = 0;
int currentNumberOfFish = 0;

float pi = 3.14159f;

foodParticle_t food[N_FOODPARTICLES];
BonyFish * fishes[N_FISHES] = {
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish
};

float magnitude (b2Vec2 vector) {
	return sqrt( (vector.x * vector.x) + (vector.y * vector.y));
}

// https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p) {
	float s = sin(angle);
	float c = cos(angle);

	// translate point back to origin:
	p.x -= cx;
	p.y -= cy;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + cx;
	p.y = ynew + cy;
	return b2Vec2(p.x,p.y);
};

JointUserData::JointUserData( b2World * m_world, b2ParticleSystem * m_particleSystem) {
	torque = 0.0f; 	
	speed = 0.0f; 	
	speedLimit = 0.0f;

	driveCW = false;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	driveCCW = false;	// a signal that tells the motor to turn in the other direction.

	upperAngle = 0.0f;
	normalAngle = 0.0f;
	lowerAngle = 0.0f;

	// init = false;
	isUsed = false;

	// the following code prepares the box2d objects.
	jointDef.bodyA = attachedTo->p_body;
	jointDef.bodyB = attaches->p_body;
	jointDef.localAnchorA =  attachedTo->tipCenter;
	jointDef.localAnchorB =  attachedTo->tipCenter;
	jointDef.enableLimit = true;
	jointDef.lowerAngle = lowerAngle;
	jointDef.upperAngle = upperAngle;
	jointDef.enableMotor = true;
    jointDef.maxMotorTorque = torque;
    jointDef.motorSpeed = speed;

    jointDef.userData = this;
    // joint = jointDef; 	// the joint that this user data struct gets pinned to

    init = true;
}

BoneUserData::BoneUserData(b2World * m_world, b2ParticleSystem * m_particleSystem) {

	// initialize everything to default, sane values
	length = 0.1f;
	rootThickness = 0.1f;
	tipThickness = 0.1f;
	density = 1.0f;

	tipCenter = b2Vec2(0.0f,0.1f); 	// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 			

	isRoot = false;
	isMouth = false;
	isSensor = false ;
	sensation = 0.0f;
	isWeapon  = false;				// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.

	energy = 0.0f; 					// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	float angle = joint->normalAngle;

	b2Vec2 tipCenter = b2Vec2(attachedTo->tipCenter.x, attachedTo->tipCenter.y + length);
	b2Vec2 rootVertexA = b2Vec2(attachedTo->tipCenter.x + (rootThickness/2), attachedTo->tipCenter.y);
	b2Vec2 rootVertexB = b2Vec2(attachedTo->tipCenter.x - (rootThickness/2), attachedTo->tipCenter.y);
	b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
	b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

	int count = 4;
	b2Vec2 vertices[count];
	vertices[0].Set(rootVertexB.x, rootVertexB.y);
	vertices[1].Set(tipVertexB.x, tipVertexB.y);
	vertices[2].Set(tipVertexA.x, tipVertexB.y);
	vertices[3].Set(rootVertexA.x, rootVertexA.y);

	// figure out the center point.
	b2Vec2 boneCenter = b2Vec2(attachedTo->tipCenter.x, attachedTo->tipCenter.y + (2*length));

	// attach user data to the body
	bodyDef.userData = this;

	bodyDef.type = b2_dynamicBody;
	p_body = m_world->CreateBody(&bodyDef);
	
	shape.SetAsBox(rootThickness, length, boneCenter, angle);	

	// reference the physics object from the user data.
	tipCenter = tipCenter;
	rootCenter = attachedTo->tipCenter;

	if (!isRoot) {
		joint = new JointUserData( m_world, m_particleSystem); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;
};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.
	// m_particleSystem->DestroyParticlesInShape( &(p_bone->shape) ,p_bone->body->GetTransform());
	if (!p_bone->isRoot) {
            p_bone->joint->isUsed = true;
			p_bone->joint->p_joint = (b2RevoluteJoint*)m_world->CreateJoint( &(p_bone->joint->jointDef) );
	}
	p_bone->isUsed = true;
}

void nonRecursiveSensorUpdater (BoneUserData * p_bone) {
	p_bone->position = p_bone->p_body->GetPosition();
	p_bone->sensation = 0.0f;

	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (food[i].init) {
			
			b2Vec2 positionalDifference = b2Vec2((p_bone->position.x - food[i].position.x),(p_bone->position.y - food[i].position.y));

			float distance = magnitude (positionalDifference);
			if (distance > 0) {
				p_bone->sensation += 1/distance;
			}
		}
	}
	printf("sensation: %f\n", p_bone->sensation);
}

// add a food particle to the world and register it so the game knows it exists.
void addFoodParticle ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	food[currentNumberOfFood].energy = 1.0f;

	b2BodyDef bd9;
	bd9.userData = &food[currentNumberOfFood]; // register the fishfood struct as user data on the body
	bd9.type = b2_dynamicBody;
	b2Body* body9 = m_world->CreateBody(&bd9);
	b2CircleShape shape9;
	// shape9.SetUserData(&fishFood)	// and also on the shape
	shape9.m_p.Set(-position.x,position.y );
	shape9.m_radius = 0.02f;
	body9->CreateFixture(&shape9, 4.0f);
	m_particleSystem->DestroyParticlesInShape(shape9,body9->GetTransform()); // snip out the particles that are already in that spot so it doesn't explode

	food[currentNumberOfFood].p_body = body9;
	food[currentNumberOfFood].shape = shape9;

	food[currentNumberOfFood].init = true;

	currentNumberOfFood++;
}

BonyFish::BonyFish(b2World * m_world, b2ParticleSystem * m_particleSystem)
{
	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating
	position = b2Vec2(0.0f, 0.0f); // the starting position of the fish in the game world

	for (int i = 0; i < N_FINGERS; ++i) {
		bones[i] = new BoneUserData( m_world, m_particleSystem);
	}
	init = true; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false;

}

// this describes the original 3 boned jellyfish.
// void makeAJellyfish (BonyFish * p_fish, b2World * m_world, b2ParticleSystem * m_particleSystem) {

// p_fish->bones[0]->joint = {
// 	1.0f,
// 	0.0f,
// 	2.0f,
// 	false,
// 	false,
// 	0.0f,
// 	0.0f * pi,
// 	0.0f,
// 	nullptr,
// 	false,
// 	true
// };

// p_fish->bones[1]->joint = {
// 	1.0f,
// 	0.0f,
// 	2.0f,
// 	false,
// 	false,
// 	0.05f,
// 	-0.15f * pi,
// 	0.25f,
// 	nullptr,
// 	false,
// 	true
// };

// p_fish->bones[2]->joint = {
// 	1.0f,
// 	0.0f,
// 	2.0f,
// 	false,
// 	false,
// 	0.05f,
// 	0.15f * pi,
// 	0.25f,
// 	nullptr,
// 	false,
// 	true
// };

// p_fish->bones[0] = {
// 	0.15f,
// 	0.01f,
// 	0.01f,
// 	1.5f,
// 	b2Vec2(0,0),
// 	b2Vec2(0,0),
// 	&joint3,
// 	nullptr,
// 	true,
// 	true,
// 	true,
// 	1.0f,
// 	false,
// 	0.0f,
// 	nullptr,
// 	nullptr,
// 	b2Vec2(0,0),
// 	false,
// 	true
// };

// p_fish->bones[1] = {
// 	0.1f,
// 	0.01f,
// 	0.01f,
// 	1.5f,
// 	b2Vec2(0,0),
// 	b2Vec2(0,0),
// 	&joint1,
// 	&bone0,
// 	false,
// 	false,
// 	false,
// 	0.0f,
// 	false,
// 	0.0f,
// 	nullptr,
// 	nullptr,
// 	b2Vec2(0,0),
// 	false,
// 	true
// };

// p_fish->bones[2]= {
// 	0.1f,
// 	0.01f,
// 	0.01f,
// 	1.5f,
// 	b2Vec2(0,0),
// 	b2Vec2(0,0),
// 	&joint2,
// 	&bone0,
// 	false,
// 	false,
// 	false,
// 	0.0f,
// 	false,
// 	0.0f,
// 	nullptr,
// 	nullptr,
// 	b2Vec2(0,0),
// 	false,
// 	true
// };

// p_fish = {
// 	{bone0,bone1,bone2,nullptr,nullptr,nullptr,nullptr,nullptr},
// 	0.0f,
// 	b2Vec2(0.0f,2.5f),
// 	false,
// 	true
// }; 

// for (int i = 0; i < N_FINGERS; ++i)
// {
// 	if (p_fish->bones[i].isUsed) {
// 		nonRecursiveBoneIncorporator(p_fish->bones[i], m_world, m_particleSystem) ;
// 	}
// }

// p_fish->init = true;
// p_fish->isUsed = true;

// }

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem) {

	// // add a food particle to the game world
	addFoodParticle(b2Vec2(2.5f, 2.5f), m_world, m_particleSystem);

	// initialize 10 fish

	// makeAJellyfish(&fishes[0], m_world, m_particleSystem);

}


void deepSeaLoop () {

	// get positions of all the food particles
	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (food[i].init) {
			food[i].position = food[i].p_body->GetPosition();
		}
	}

	// for each fish in the game world
	for (int i = 0; i < N_FISHES; i++) {
		if (fishes[i]->init) {

			// iterate through the bones of the fish and update any sensors
			for (int j = 0; j < N_FINGERS; ++j)
			{
				// nonRecursiveSensorUpdater( &(fishes[i]->bones[j]));
			}


		

		// sensory detection of food
		// each sensor on the animal simply indicates a float number which is the amount of food it 'smells'. it does not tell direction and cannot distinguish different smell sources

		// running of the brain or behavior algorithm

		// motor outputs or other controls


		}
	}
}