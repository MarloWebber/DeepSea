# Some flags for Freeglut and GLUI.
add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC
                 -DGLUT_DISABLE_ATEXIT_HACK -D_CRT_SECURE_NO_WARNINGS )

# https://bytefreaks.net/programming-2/cc-how-do-you-set-gdb-debug-flag-g-with-cmake
# set(CMAKE_BUILD_TYPE Debug)

# Define the framework files.
set(Testbed_Framework_SRCS
	Framework/Main.cpp
	Framework/Main.h
	Framework/Render.cpp
	Framework/Render.h
	Framework/Test.cpp
	Framework/Test.h
	Framework/Arrow.cpp
	Framework/Arrow.h
	Framework/FullscreenUI.cpp
	Framework/FullscreenUI.h
	Framework/ParticleParameter.cpp
	Framework/ParticleParameter.h
	Framework/DeepSea.cpp
	Framework/DeepSea.h
)

#define the test files.
set(Testbed_Tests_SRCS
	Tests/TestEntries.cpp
	Tests/AddPair.h
	Tests/ApplyForce.h
	Tests/BodyTypes.h
	Tests/Breakable.h
	Tests/Bridge.h
	Tests/BulletTest.h
	Tests/Cantilever.h
	Tests/Car.h
	Tests/Chain.h
	Tests/CharacterCollision.h
	Tests/CollisionFiltering.h
	Tests/CollisionProcessing.h
	Tests/CompoundShapes.h
	Tests/Confined.h
	Tests/ContinuousTest.h
	Tests/ConvexHull.h
	Tests/ConveyorBelt.h
	Tests/DistanceTest.h
	Tests/Dominos.h
	Tests/DumpShell.h
	Tests/DynamicTreeTest.h
	Tests/EdgeShapes.h
	Tests/EdgeTest.h
	Tests/Gears.h
	Tests/Mobile.h
	Tests/MobileBalanced.h
	Tests/MotorJoint.h
	Tests/OneSidedPlatform.h
	Tests/Pinball.h
	Tests/PolyCollision.h
	Tests/PolyShapes.h
	Tests/Prismatic.h
	Tests/Pulleys.h
	Tests/Pyramid.h
	Tests/RayCast.h
	Tests/Revolute.h
	Tests/Rope.h
	Tests/RopeJoint.h
	Tests/SensorTest.h
	Tests/ShapeEditing.h
	Tests/SliderCrank.h
	Tests/SphereStack.h
	Tests/TheoJansen.h
	Tests/Tiles.h
	Tests/TimeOfImpact.h
	Tests/Tumbler.h
	Tests/VaryingFriction.h
	Tests/VaryingRestitution.h
	Tests/VerticalStack.h
	Tests/Web.h

	Tests/AntiPointy.h
	Tests/CornerCase.h
	Tests/DamBreak.h
	Tests/DrawingParticles.h
	Tests/ElasticParticles.h
	Tests/Faucet.h
	Tests/Fracker.h
	Tests/Impulse.h
	Tests/LiquidTimer.h
	Tests/Maxwell.h
	Tests/MultipleParticleSystems.h
	Tests/Particles.h
	Tests/ParticlesSurfaceTension.h
	Tests/Pointy.h
	Tests/Ramp.h
	Tests/RigidParticles.h
	Tests/Sandbox.h
	Tests/Soup.h
	Tests/SoupStirrer.h
	Tests/Sparky.h
	Tests/WaveMachine.h
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

include_directories (
	${OPENGL_INCLUDE_DIR}
	${Box2D_SOURCE_DIR}
	../../../freeglut/include
)

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib/ )
	find_library(OPENGL_LIBRARIES NAMES GL GLU X11 HINTS /usr/X11/lib)
elseif(WIN32)
	set (ADDITIONAL_LIBRARIES winmm)
endif(APPLE)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

target_link_libraries (
	Testbed
	Box2D
	freeglut_static
	glui
	fann
	${ADDITIONAL_LIBRARIES}
	${OPENGL_LIBRARIES}
)
