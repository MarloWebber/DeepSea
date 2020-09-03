




		// liquidfun structs contain a place for a pointer to user data.
		// for each different kind of liquidfun struct i use, i should create a corresponding user data struct that pins it into my game.
		// these are going to get generated first, by the game logic



		// prototypes
		class BonyFish;
		// typedef struct SquishyFish;

		
		// 
		typedef struct jointUserData {

			float torque; 	
			float speed; 	
			float speedLimit;

			bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
			bool driveCCW;	// a signal that tells the motor to turn in the other direction.

			float upperAngle;
			float normalAngle;
			float lowerAngle;

			b2RevoluteJoint joint; // the joint that this user data struct gets pinned to
		}



		class boneUserData {
			// this will comprise the organs of the rigid animals.

			float length;
			float rootThickness;
			float tipThickness;

			float density;


			jointUserData joint; // the joint that attaches it into its socket			

			boneUserData bones[8]; // a collection of the other bones that are attached to it.

			bool isSensor = false;
			float foodSensor = 0.0f;


			b2Body body;
			b2PolygonShape shape; 



		}


		
		// typedef struct particleGroupUserData {
		// 	// this will typically comprise organs of the squishy animals.

		// }

		// https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
		b2vec2 rotatePoint(float cx,float cy,float angle, b2vec2 p)
		{
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
		  return b2vec2(p.x,p.y);
		}


	void recursiveBoneIncorporator(bone, cumulativeBonePosition) {

			// generate the b2 vectors for each bone
						// first, the center of the root of the bone will be at cumulativeBonePosition.
						// then, the vertexes on the root are placed 1/2 the width to either side, and rotated by the bone angle.


						b2vec2 rootVertexA = b2vec2(cumulativeBonePosition[0] + (bone.rootThickness/2), cumulativeBonePosition[1]);
						b2vec2 rootVertexB = b2vec2(cumulativeBonePosition[0] - (bone.rootThickness/2), cumulativeBonePosition[1]);

						rootVertexA = rotatePoint( cumulativeBonePosition[0], cumulativeBonePosition[1], bone.normalAngle, rootVertexA);
						rootVertexB = rotatePoint( cumulativeBonePosition[0], cumulativeBonePosition[1], bone.normalAngle, rootVertexB);

						// then you figure out the position of the tip center and do it again for the vertices at the end of the bone.
						b2vec2 tipCenter = b2vec2(cumulativeBonePosition[0], cumulativeBonePosition[1] + bone.length);
						tipCenter = rotatePoint( cumulativeBonePosition[0], cumulativeBonePosition[1], bone.normalAngle, tipCenter);


						b2vec2 tipVertexA = b2vec2(tipCenter[0] + (bone.tipThickness/2), tipCenter[1]);
						b2vec2 tipVertexB = b2vec2(tipCenter[0] - (bone.tipThickness/2), tipCenter[1]);

						tipVertexA = rotatePoint( tipCenter[0], tipCenter[1], bone.normalAngle, tipVertexA);
						tipVertexB = rotatePoint( tipCenter[0], tipCenter[1], bone.normalAngle, tipVertexB);


						b2vec2 vertices[] = {rootVertexB, tipVertexB, tipVertexA, rootVertexA};

						// attach pointers to the b2 structs into the user data object and vice versa. for b2body this has to be done before you take the bodydef object to the factory				
					// generate the b2 bodies and shapes. 
						b2BodyDef bd1;

						b1.SetUserData(&bone)

						bd1.type = b2_dynamicBody;
						b2Body* body1 = m_world->CreateBody(&bd1);
						b2PolygonShape shape1;

						shape1.SetUserData(&bone)

						// they are added into the world
						shape1.set(vertices); //SetAsBox(0.01f, 0.15f, b2Vec2(2.0f,2.5f+ (i * 0.3)), 0.0f);
						body1->CreateFixture(&shape1, 1.5f);
						m_particleSystem->DestroyParticlesInShape(shape1,
																  body1->GetTransform());

						bone.body = body1;
						bone.shape = shape1;


						// run this same function on any new bones that the bone had.
						n_bones = len(bone.bones);
						for (int i = 0; i < n_bones, i++) {

							recursiveBoneIncorporator(bone, tipCenter);
							
						}

			}





		class BonyFish {
			// these are the animals made from rigid physical objects in the game world.
			// they are comprised of skeletons that brachiate from an origin point. each bone ends in none, one, or several others. bones can be jointed and can move to apply force.
			// these have the potential to be simple to implement.

			// let us say that the bone arrays can be capped to 8 for now.
			public:
				b2boneUserData bones[8];

			// a neural network brain

			// the starting position of the fish in the game world
				b2vec2 position = b2vec2(0.0f, 0.0f);



		
			void incorporate () {

				// for each bone

				b2vec2 cumulativeBonePosition = position;

				n_bones = len(bones);
				for (int i = 0; i < n_bones, i++) {



					b2vec2 boneEnd = recursiveBoneIncorporator(bone);



					// deploy them into the world
				}

				// for each joint
				// figure out the joint position
				// create the b2 joint object
				// reference it to the user data object
				// insert the joint into the game world

			}


			private:



		}

		// typedef struct SquishyFish {
		// 	// this is the type of animal made of particle groups.
		// 	// liquidfun already provides a range of softbody materials that is enough to construct simple animal bodies, with stiff parts, flexible and elastic parts.
		// 	// these have the potential to be cuter and more hilarious than the very serious rigid-body animals.
		// 	// particle groups can be made into finer and more detailed shapes than the rigid bones.
		// 	// to create a muscle, we must provide force onto the tissue using liquidfun's force tool. we have to figure that out ourselves.
		// }


		// this describes the original 3 boned jellyfish.

				boneUserData bone1 = {
					.length = 0.1f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint1 = {
						.torque = 1.0f;
						.speed = 0.0f;
						.speedLimit = 2.0f;
						.upperAngle = 0.05f;
						.lowerAngle = 0.25f;
					}
					.bones = {};
				};

				boneUserData bone2 = {
					.length = 0.1f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint2 = {
						.torque = 1.0f;
						.speed = 0.0f;
						.speedLimit = 2.0f;
						.upperAngle = 0.25f;
						.lowerAngle = 0.05f;
					}
					.bones = {};
				};

				boneUserData bone0 = {
					.length = 0.15f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint0 = {
						.torque = 0.0f;
						.speed = 0.0f;
						.speedLimit = 0.0f;
						.upperAngle = 0.0f;
						.lowerAngle = 0.0f;
					}
					.bones = {bone1, bone2};
				};

				BonyFish simpleJellyfish;

				simpleJellyfish.bones = {bone0};






		BonyFish[] fishInTheSea = {
			simpleJellyfish
		}


		// methods that are run on bony fish.


		// from a bonyfish struct, generate a serialized fish.
		// recreate a struct from a serialized fish.


		








