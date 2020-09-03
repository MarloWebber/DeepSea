
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


void recursiveBoneIncorporator(boneUserData_t*  p_bone, b2Vec2 cumulativeBonePosition, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	/*
	this is intended to be a recursive function that creates the heirarchical bone animals. this function is run manually for the first array of bones and then autoruns subsequently for their children bones.

	cumulative bone position is the tip of the previous bone, i.e. the working area for adding new ones.
	*/

	// generate the b2 vectors for each bone
	// first, the center of the root of the bone will be at cumulativeBonePosition.
	// then, the vertexes on the root are placed 1/2 the width to either side, and rotated by the bone angle.

	boneUserData_t bone = *p_bone;

	float angle = bone.joint->normalAngle;

	b2Vec2 rootVertexA = b2Vec2(cumulativeBonePosition.x + (bone.rootThickness/2), cumulativeBonePosition.y);
	b2Vec2 rootVertexB = b2Vec2(cumulativeBonePosition.x - (bone.rootThickness/2), cumulativeBonePosition.y);
	rootVertexA = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexA);
	rootVertexB = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexB);

	// then you figure out the position of the tip center and do it again for the vertices at the end of the bone.
	b2Vec2 tipCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + bone.length);
	tipCenter = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, tipCenter);

	b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (bone.tipThickness/2), tipCenter.y);
	b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (bone.tipThickness/2), tipCenter.y);
	tipVertexA = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexA);
	tipVertexB = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexB);

	b2Vec2 vertices[4] = {rootVertexB, tipVertexB, tipVertexA, rootVertexA};

	// attach pointers to the b2 structs into the user data object and vice versa. for b2body this has to be done before you take the bodydef object to the factory				
	// generate the b2 bodies and shapes. 
	b2BodyDef bd1;

	bd1.userData = &bone;

	bd1.type = b2_dynamicBody;
	b2Body* body1 = m_world->CreateBody(&bd1);
	b2PolygonShape shape1;

	// shape1.SetUserData(&bone)

	// they are added into the world
	shape1.Set(vertices, 4);
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
	
	// run this same function on any new bones that the bone had.
	for (int i = 0; i < bone.n_bones; i++) {
		boneUserData_t * p_sub_bone = (boneUserData_t*)bone.bones[i];
		recursiveBoneIncorporator(p_sub_bone, tipCenter, m_world, m_particleSystem);
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







