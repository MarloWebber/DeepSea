#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

int currentNumberOfFood = 0;
int currentNumberOfFish = 0;

bool fishSlotLoaded[8];

float pi = 3.14159f;

foodParticle_t food[N_FOODPARTICLES];
BonyFish * fishes[N_FISHES];

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

JointUserData::JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	torque = boneDescription.torque; 	
	speed = 0.0f; 	
	speedLimit = boneDescription.speedLimit;
	upperAngle = boneDescription.upperAngle;
	normalAngle = boneDescription.normalAngle;
	lowerAngle = boneDescription.lowerAngle;

	driveCW = false;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	driveCCW = false;	// a signal that tells the motor to turn in the other direction.

	// init = false;
	isUsed = false;

	// the following code prepares the box2d objects.
	if (boneDescription.isRoot) {
		// jointDef.bodyA = attachedTo->p_body;
		// jointDef.bodyB = attaches->p_body;
		// jointDef.localAnchorA =  attachedTo->tipCenter;
		// jointDef.localAnchorB =  attachedTo->tipCenter;
		// jointDef.enableLimit = true;
		// jointDef.lowerAngle = lowerAngle;
		// jointDef.upperAngle = upperAngle;
		// jointDef.enableMotor = true;
	 //    jointDef.maxMotorTorque = torque;
	 //    jointDef.motorSpeed = speed;
	}
	else {
		jointDef.bodyA = fish->bones[boneDescription.attachedTo]->p_body;
		jointDef.bodyB = p_bone->p_body;
		jointDef.localAnchorA =  fish->bones[boneDescription.attachedTo]->tipCenter;
		jointDef.localAnchorB =  fish->bones[boneDescription.attachedTo]->tipCenter;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = lowerAngle;
		jointDef.upperAngle = upperAngle;
		jointDef.enableMotor = true;
	    jointDef.maxMotorTorque = torque;
	    jointDef.motorSpeed = speed;
	}
	

    jointDef.userData = this;
    // joint = jointDef; 	// the joint that this user data struct gets pinned to

    init = true;
}

BoneUserData::BoneUserData(
	//	 these are the parameters that can fully describe a bone, in condensed form

		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2World * m_world, b2ParticleSystem * m_particleSystem // primarily needed to create the body
	) {

	if (!boneDescription.used) {
		return;
	}

	BoneUserData * attachesTo = fish->bones[boneDescription.attachedTo];	

	// initialize everything to default, sane values
	length = boneDescription.length;
	rootThickness = boneDescription.rootThickness;
	tipThickness = boneDescription.tipThickness;
	density = 1.0f; //boneDescription.density;
	isRoot = boneDescription.isRoot;
	isMouth = boneDescription.isMouth;
	isSensor = boneDescription.isSensor;
	sensation = 0.0f;
	isWeapon  = boneDescription.isWeapon;				// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * (length * density); 					// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	tipCenter = b2Vec2(0.0f,0.1f); 	// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 		

	// the following code is used to generate box2d structures and shapes from the bone parameters.

	if (isRoot) {

		b2Vec2 tipCenter = b2Vec2(0.0f, 0.0f + length);
		b2Vec2 rootVertexA = b2Vec2(0.0f + (rootThickness/2), 0.0f);
		b2Vec2 rootVertexB = b2Vec2(0.0f - (rootThickness/2), 0.0f);

		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

		int count = 4;
		b2Vec2 vertices[count];
		vertices[0].Set(rootVertexB.x, rootVertexB.y);
		vertices[1].Set(tipVertexB.x, tipVertexB.y);
		vertices[2].Set(tipVertexA.x, tipVertexB.y);
		vertices[3].Set(rootVertexA.x, rootVertexA.y);

		// figure out the center point.
		b2Vec2 boneCenter = b2Vec2(0.0f, 0.0f + (2*length));

		// attach user data to the body
		bodyDef.userData = this;

		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);
		
		shape.SetAsBox(rootThickness, length, boneCenter, 0.0f);	

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {
		b2Vec2 tipCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + length);
		b2Vec2 rootVertexA = b2Vec2(attachesTo->tipCenter.x + (rootThickness/2), attachesTo->tipCenter.y);
		b2Vec2 rootVertexB = b2Vec2(attachesTo->tipCenter.x - (rootThickness/2), attachesTo->tipCenter.y);

		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

		int count = 4;
		b2Vec2 vertices[count];
		vertices[0].Set(rootVertexB.x, rootVertexB.y);
		vertices[1].Set(tipVertexB.x, tipVertexB.y);
		vertices[2].Set(tipVertexA.x, tipVertexB.y);
		vertices[3].Set(rootVertexA.x, rootVertexA.y);

		// figure out the center point.
		b2Vec2 boneCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + (2*length));

		// attach user data to the body
		bodyDef.userData = this;

		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);
		
		shape.SetAsBox(rootThickness, length, boneCenter, 0.0f);	

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = attachesTo->tipCenter;

	}

	if (!isRoot) {
		joint = new JointUserData( boneDescription, this, fish, m_world, m_particleSystem); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;
};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	if (!p_bone->init) {
		return;
	}
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
};

void brainalyzer () {
    const unsigned int num_input = 4; // should be number of sensors and hearts
    const unsigned int num_output = 5;
    const unsigned int num_layers = 3;
    const unsigned int num_neurons_hidden = 3;
    const float desired_error = (const float) 0.01;
    const unsigned int max_epochs = 50000;
    const unsigned int epochs_between_reports = 1000;

    struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

    fann_train_on_file(ann, "jellyfishMindControl.data", max_epochs, epochs_between_reports, desired_error);

    fann_save(ann, "jellyfishMindControl.net");

    // fann_destroy(ann);

    return;
}

BonyFish::BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating
	// position = b2Vec2(0.0f, 0.0f); // the starting position of the fish in the game world

	for (int i = 0; i < N_FINGERS; ++i) {

		if (i == 0) {
			// set the bone as root
			driedFish.bones[i].isRoot = true;
		}

		bones[i] = new BoneUserData(driedFish.bones[i], this,  m_world, m_particleSystem);
	}

	init = true; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false;

	brainalyzer();

};

// this describes the original 3 boned jellyfish.
fishDescriptor_t simpleJellyfish = {
	{
		{
				0,		// attachesTo
				0.15f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				true,	// isRoot
				true,	// isMouth
				true,	// isSensor
				false,	// isWeapon
				0.0f,	// torque
				0.0f,	// speedLimit
				0.0f,	// upperAngle
				0.0f,	// normalAngle
				0.0f,	// lowerAngle
				true
		},
		{
				0,		// attachesTo
				0.15f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				1.0f,	// torque
				1.0f,	// speedLimit
				-1.00f,	// upperAngle
				-0.5f,	// normalAngle
				-0.0f,	// lowerAngle
				true
		},
		 {
				0,		// attachesTo
				0.15f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				1.0f,	// torque
				1.0f,	// speedLimit
				0.0f,	// upperAngle
				0.5f,	// normalAngle
				1.0f,	// lowerAngle
				true
		}
	}

};

void totalFishIncorporator (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	for (int i = 0; i < N_FINGERS; ++i)
	{
		if (fishes[fishIndex]->bones[i]->init) {
			nonRecursiveBoneIncorporator( fishes[fishIndex]->bones[i] , m_world, m_particleSystem);
		}
	}
}


void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	fishes[fishIndex] = new BonyFish(driedFish, fishIndex , m_world, m_particleSystem);
	fishSlotLoaded[fishIndex] = true;

}

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem) {

	addFoodParticle(b2Vec2(2.5f, 2.5f), m_world, m_particleSystem);
// 
	loadFish(0, simpleJellyfish, m_world, m_particleSystem);

	totalFishIncorporator(0, m_world, m_particleSystem);
}


void deepSeaLoop () {

	// get positions of all the food particles
	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (food[i].init) {
			food[i].position = food[i].p_body->GetPosition();
		}
	}


	// set all the drive speeds on the fish joints.
	for (int i = 0; i < N_FISHES; ++i)

	{

		if (!fishSlotLoaded[i]) {
			return;
		}
		for (int j = 0; j < N_FINGERS; ++j)
		{
			// if (fishes[i]->bones[j]->init && fishes[i]->bones[j]->isUsed) {

			
			// 		if (fishes[i]->bones[j]->joint->driveCW) {
			// 			fishes[i]->bones[j]->joint->p_joint->SetMotorSpeed(fishes[i]->bones[j]->joint->speedLimit);
			// 		}
			// 		else if (fishes[i]->bones[j]->joint->driveCCW) {
			// 			fishes[i]->bones[j]->joint->p_joint->SetMotorSpeed(-(fishes[i]->bones[j]->joint->speedLimit));
			// 		}
			// 		else {
			// 			fishes[i]->bones[j]->joint->p_joint->SetMotorSpeed (0);
			// 		}
			// 	}
			}
		}
	

	// set all the drive signals false again.

}

void deepSeaControlA () {
	fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(1.0f);
	// printf("deepSeaControlA\n");

}
void deepSeaControlB () {
	fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(-1.0f);
}