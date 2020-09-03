
#include "DeepSea.h"

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


void recursiveBoneIncorporator(boneUserData_t*  p_bone, b2Vec2 cumulativeBonePosition, b2World * m_world, b2ParticleSystem * m_particleSystem, boneUserData_t * previousBone) {

	/*
	this is intended to be a recursive function that creates the heirarchical bone animals. this function is run manually for the first array of bones and then autoruns subsequently for their children bones.

	cumulative bone position is the tip of the previous bone, i.e. the working area for adding new ones.
	*/

	// generate the b2 vectors for each bone
	// first, the center of the root of the bone will be at cumulativeBonePosition.
	// then, the vertexes on the root are placed 1/2 the width to either side, and rotated by the bone angle.
	boneUserData_t bone = *p_bone;

	float angle = bone.joint->normalAngle;

	// then you figure out the position of the tip center and do it again for the vertices at the end of the bone.
	b2Vec2 tipCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + bone.length);
	tipCenter = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, tipCenter);


	if (false) { // saving this until i can make polygons properly.
		

		b2Vec2 rootVertexA = b2Vec2(cumulativeBonePosition.x + (bone.rootThickness/2), cumulativeBonePosition.y);
		b2Vec2 rootVertexB = b2Vec2(cumulativeBonePosition.x - (bone.rootThickness/2), cumulativeBonePosition.y);
		rootVertexA = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexA);
		rootVertexB = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexB);

	
		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (bone.tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (bone.tipThickness/2), tipCenter.y);
		tipVertexA = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexA);
		tipVertexB = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexB);
	}
	

	// b2Vec2 vertices[4];
	// vertices[0].Set(rootVertexB.x, rootVertexB.y);
	// vertices[1].Set(tipVertexB.x, tipVertexB.y);
	// vertices[2].Set(tipVertexA.x, tipVertexB.y);
	// vertices[3].Set(rootVertexA.x, rootVertexA.y);


	// b2Vec2 vertices[4] = {	b2Vec2(rootVertexB.x, rootVertexB.y),
	// 						b2Vec2(tipVertexB.x, tipVertexB.y),
	// 						b2Vec2(tipVertexA.x, tipVertexA.y),
	// 						b2Vec2(0.1f, 0.5f), };


	// printf("RootA.x: %f, RootA.y: %f\n", vertices[3].x, vertices[3].y);
	// printf("RootB.x: %f, RootB.y: %f\n", vertices[0].x, vertices[0].y);
	// printf("tipA.x: %f, tipA.y: %f\n", vertices[2].x, vertices[2].y);
	// printf("tipB.x: %f, tipB.y: %f\n", vertices[1].x, vertices[1].y);
	// printf("---\n");

	// it's okay. we can use setasbox instead.

	// figure out the center point.
	b2Vec2 boneCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + (bone.length/2));
	boneCenter = rotatePoint(cumulativeBonePosition.x, cumulativeBonePosition.y, angle, boneCenter);

	// attach pointers to the b2 structs into the user data object and vice versa. for b2body this has to be done before you take the bodydef object to the factory				
	// generate the b2 bodies and shapes. 
	b2BodyDef bd1;

	bd1.userData = &bone;

	bd1.type = b2_dynamicBody;
	b2Body* body1 = m_world->CreateBody(&bd1);
	b2PolygonShape shape1;

	// shape1.SetUserData(&bone)

	// they are added into the world
	// shape1.Set(vertices, 4);
	shape1.SetAsBox(bone.rootThickness, bone.length, boneCenter, angle);	
	body1->CreateFixture(&shape1, 1.5f);
	m_particleSystem->DestroyParticlesInShape(shape1,
											  body1->GetTransform());

	bone.body = body1;
	bone.shape = &shape1;

	bone.tipCenter = tipCenter;
	bone.rootCenter = cumulativeBonePosition;

	// create the joint that pins it to the root bone.

				// for each joint
				// figure out the joint position
				// create the b2 joint object
				// reference it to the user data object
				// insert the joint into the game world


	if (previousBone != nullptr) {
			b2RevoluteJointDef jointDef2;
			jointDef2.bodyA = previousBone->body;
			jointDef2.bodyB = bone.body;
			jointDef2.localAnchorA =  cumulativeBonePosition; //SetZero();
			jointDef2.localAnchorB =  cumulativeBonePosition;//SetZero();
			// jointDef1.frequencyHz = 100.0f;
			// jointDef1.dampingRatio = 0.0f;
			// jointDef1.length = 0.0f;
			jointDef2.enableLimit = true;
			jointDef2.lowerAngle = bone.joint->lowerAngle;
			jointDef2.upperAngle = bone.joint->upperAngle;

			jointDef2.enableMotor = true;
            jointDef2.maxMotorTorque = 1.0f;
            jointDef2.motorSpeed = 0.0f;

            jointDef2.userData = &bone.joint;

			m_world->CreateJoint(&jointDef2);
       
	}
	
	// run this same function on any new bones that the bone had.
	for (int i = 0; i < bone.n_bones; i++) {
		boneUserData_t * p_sub_bone = (boneUserData_t*)bone.bones[i];
		recursiveBoneIncorporator(p_sub_bone, tipCenter, m_world, m_particleSystem, &bone);
	};

	// set cumulative bone position back to the root, so that it is in the right place for the next lot of sub bones.
	cumulativeBonePosition = bone.rootCenter;

	// that was my favorite episode of bones.
};

// typedef struct SquishyFish {
// 	// this is the type of animal made of particle groups.
// 	// liquidfun already provides a range of softbody materials that is enough to construct simple animal bodies, with stiff parts, flexible and elastic parts.
// 	// these have the potential to be cuter and more hilarious than the very serious rigid-body animals.
// 	// particle groups can be made into finer and more detailed shapes than the rigid bones.
// 	// to create a muscle, we must provide force onto the tissue using liquidfun's force tool. we have to figure that out ourselves.
// }







