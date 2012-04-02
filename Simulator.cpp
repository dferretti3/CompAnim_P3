/*
 *  Simulator.cpp
 *  BasicPhyx
 *
 *  Created by Karen Liu on 12/11/10.
 *  Copyright 2010 GA Tech. All rights reserved.
 *
 */

#ifndef __SIMULATOR_H__
#include "Simulator.h"
#endif

#include "Util/UserAllocator.h"
#include "Util/Timing.h"
#include "Util/Utilities.h"
#include "DrawObjects.h"
#include "Actors.h"

NxVec3 Simulator::ApplyForceToActor(NxActor *actor, const NxVec3& forceDir)
{
	NxVec3 forceVec = mForceStrength * forceDir;
	
	if (mForceMode)
		actor->addForce(forceVec);
	else 
		actor->addTorque(forceVec);
	
	return forceVec;
}

bool shootPressed = false;
bool showTrajBall = true;
bool bPressed = false;
void Simulator::ProcessKeys(const bool *keys, NxVec3 gCamForward)
{	
	if (keys[' ']) { if (shootPressed) holdSpace( gCamForward); else pressSpace( gCamForward); shootPressed=true;}
	else { if (shootPressed) releaseSpace( gCamForward); shootPressed=false;}

	if (keys['b']) {if (!bPressed) showTrajBall = !showTrajBall; bPressed = true;}
	else {bPressed = false;}
}

const double PI = 3.1415926;
NxVec3 targetPoint = NxVec3(0,0,0);
NxVec3 targetNormal = NxVec3(0,0,0);
NxVec3 launch = NxVec3(0,0,0);
NxActor* trajBall = NULL;
const double t_start = -0.4;
const double t_end = 1.5;
double dt = 0.01;
double t = t_start;
NxReal radius = 0.3;
const double HEIGHT = 0.6;
const double PW = 4;


NxVec3 Simulator::getPos(NxVec3 gCamForward)
{
	NxQuat q; 
	NxMat34 rot;
	float theta = acos( NxVec3(0, 0, 1).dot(gCamForward)) * 180/NxPiF32;
	q.fromAngleAxis(theta+90, NxVec3(0, 1, 0));
	rot.M.fromQuat(q);
	NxVec3 offset = rot*NxVec3(-0.5,1.2,0.417);
	return player->getGlobalPosition()+offset;
}

NxVec3 Simulator::getLaunch(double t, NxVec3 camForward)
{
	
	return 8*camForward+4*NxVec3(0,1.2*t,0);
}

void Simulator::pressSpace(NxVec3 gCamForward)
{
	if (trajBall!=NULL) mScene->releaseActor(*trajBall);
	NxVec3 pos = getPos(gCamForward);
	trajBall = mActors->CreateEmptyBall(pos,radius);
	t = t_start;
	launch = getLaunch(t,gCamForward);
}

void Simulator::holdSpace(NxVec3 gCamForward)
{
	if (trajBall==NULL) return;
	trajBall->setGlobalPosition(getPos(gCamForward));
	t += dt;
	if (t>=t_end) {t=t_end-dt; dt*=-1;}
	if (t<=t_start){t=t_start-dt; dt*=-1;}
	launch = getLaunch(t,gCamForward);

	NxVec3 tpos = getPos(gCamForward);
	NxVec3 tvel = getLaunch(t,gCamForward);
	NxVec3 tvel_n = NxVec3(tvel);
	tvel_n.normalize();
	NxVec3 g = NxVec3(0,-9.8,0);
	NxReal dt = deltaTime;
	int MAX_I = 100;
	int segSkip = MAX_I/MAX_ARC_SEGS;
	numSegs = 0;
	NxU32 sweep = 0;
	NxU32 flags = NX_SF_STATICS | NX_SF_DYNAMICS;
	NxSweepQueryHit hit;
	double dist = 0.005;
	for (int i = 0; i < MAX_I && sweep==0; i++)
	{
		tvel += g*dt;
		tpos += tvel*dt;
		trajBall->setGlobalPosition(tpos);

		//add line segment here
		if (i%segSkip==0)
		{
			arc_segs[numSegs++] = tpos+NxVec3(0,radius,0);
		}

		//raycast here
		sweep = trajBall->linearSweep(tvel_n*dist,flags,NULL,1,&hit,NULL);
	}
	if (sweep==sweep)
	{
		//trajBall->setGlobalPosition(tpos);
		//printf("(%f,%f,%f)\n",tpos.x,tpos.y,tpos.z);
	} else
	{
		//printf("No collision predicted\n");
	}
	targetPoint = tpos;


	//NxVec3 dir = NxVec3(1,0,0)-pos;
	//dir.normalize();
	//float dist = 10.0f;
	//NxU32 sweep = ball->linearSweep(dir*dist,flags,NULL,1,&hit,NULL);
	//if (sweep)
	//{
	//	//mScene->releaseActor(hit.hitShape->getActor());
	//	targetPoint = hit.point;
	//	targetNormal = hit.normal;
	//}
}

void Simulator::releaseSpace(NxVec3 gCamForward)
{
	if (trajBall!=NULL) mScene->releaseActor(*trajBall);
	trajBall = NULL;
	double mass = 3.0;
	NxActor* ball = mActors->CreateBall(getPos(gCamForward),radius,mass);
	//ball->addForce(launch,NX_IMPULSE);
	ball->setLinearVelocity(launch);
}

void Simulator::RenderStuff()
{
	//PrintText("SCORE:",0,0,1,0,0);
	drawText(0,470,"SCORE:");

	for (int i = 0; i <4; i++)
	{
		DrawLine(PLANE[i],PLANE[(i+1)%4],NxVec3(1,1,0));
	}

	if (trajBall!=NULL)
	{
		//DrawArrow(getPos(),getPos()+0.3*launch,NxVec3(1,0,0));
		DrawContactPoint(targetPoint+NxVec3(0,radius,0),0.4,NxVec3(0,1,0));
		for (int i = 0; i < numSegs-1; i++)
		{
			DrawLine(arc_segs[i],arc_segs[i+1],NxVec3(1,0,0));
		}
	}

}

void Simulator::RenderActors()
{
    // Render all the actors in the scene
    int nActor = mScene->getNbActors();
    NxActor** actors = mScene->getActors();
    while (nActor--){
        NxActor* actor = *actors++;
        if (actor!=trajBall || showTrajBall)
			DrawActor(actor);
    }
}

void Simulator::RenderForce(NxActor* actor, NxVec3& forceVec, const NxVec3& color)
{
	// draw only if the force is large enough
	NxReal force = forceVec.magnitude();
	if (force < 0.1f)  return;
	
	forceVec = 3 * forceVec/force;
	
	NxVec3 pos = actor->getCMassGlobalPosition();
	DrawArrow(pos, pos + forceVec, color);
}

void Simulator::RenderScene()
{
	RenderActors();
	if(mActors->mSelectedActor)
		RenderForce(mActors->mSelectedActor, mForceVec, NxVec3(1, 0, 0));
	RenderStuff();
	if (win)
	{
			drawText(300,230,"WINNER!");
	}
}

void Simulator::releaseActors()
{
	int n = mScene->getNbActors();
	printf("actors:%d\n",n);
	/*for(int i = 0; i < n; i++)
		mScene->releaseActor(*mObjects[i]);*/
	/*NxActor** actors = mScene->getActors();
	for (int i = 0; i < n; i++)
	{
		mScene->releaseActor(*(actors[i]));
	}*/
    NxActor** actors = mScene->getActors();
    while (n--){
        NxActor* actor = *actors++;
		//mScene->releaseActor();
		
    }
}

void Simulator::nextLevel()
{
	levelCounter = (levelCounter+1)%3;
}

void Simulator::restartLevel()
{
	win = false;
	releaseActors();
	switch (levelCounter)
	{
	case 0: { buildCastle(); break;}
	}
}

bool Simulator::InitNx()
{
	if(mAllocator == NULL)
		mAllocator = new UserAllocator;
	
	// Initialize PhysicsSDK
	NxPhysicsSDKDesc desc;
	NxSDKCreateError errorCode = NXCE_NO_ERROR;
	mSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, mAllocator, NULL, desc, &errorCode);
	if(mSDK == NULL){
		printf("\nSDK create error (%d - %s).\nUnable to initialize the PhysX SDK, exiting the sample.\n\n", errorCode, getNxSDKCreateError(errorCode));
		return false;
	}
	
	// Set the physics parameters
	mSDK->setParameter(NX_SKIN_WIDTH, 0.005f);
	
	// Set the debug visualization parameters
	mSDK->setParameter(NX_VISUALIZATION_SCALE, 1);
	mSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1);
	mSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1);
	mSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, 1);


    // Create the scene
    NxSceneDesc sceneDesc;
    sceneDesc.gravity = NxVec3(0, -9.8, 0);
    mScene = mSDK->createScene(sceneDesc);
	if(mScene == NULL){
		printf("\nError: Unable to create a PhysX scene, exiting the sample.\n\n");
		return false;
	}
	return true;
}

void Simulator::CreateScene()
{	
	mActors = new Actors(mSDK, mScene);
	NxMaterial *defaultMaterial = mScene->getMaterialFromIndex(0); 
	defaultMaterial->setRestitution(0.0f);
	defaultMaterial->setStaticFriction(0.5f);
	defaultMaterial->setDynamicFriction(0.5f);
	
	// Create the objects in the scene
	mObjects.push_back(mActors->CreateGroundPlane());

	//create player
	player = mActors->CreateCatapult(NxVec3(5, 0, 0), NxVec3(0.5, 0.5, 0.5), 1.0);

	buildCastle();
	
	//mActors->CreateStack(NxVec3(0, 0, 0), NxVec3(2, 7, 2), NxVec3(0.2, 0.2, 0.2), 1.0);
	
	/*int levels = 5;
	NxVec3 offset(-6,0,0);
	NxReal diff = 1.20;
	for (int r = levels; r > 0; r--)
	{
		for (int i = 0; i < r; i++)
		{
			for (int j = 0; j < r; j++)
			{
				NxReal x = i*diff - r*diff/2;
				NxReal z = j*diff - r*diff/2;
				NxReal y = levels-r;
				mActors->CreateBox(NxVec3(x,y,z)+offset,NxVec3(0.5,0.5,0.5),1.0);
			}
		}
	}*/

	PLANE[0] = NxVec3(-PW,HEIGHT,PW);
	PLANE[1] = NxVec3(-PW,HEIGHT,-PW);
	PLANE[2] = NxVec3(PW,HEIGHT,-PW);
	PLANE[3] = NxVec3(PW,HEIGHT,PW);
	
	mActors->mSelectedActor = player;
	getElapsedTime();
}

void Simulator::myBuildStack(NxVec3 pos,NxVec3 stackDim,NxVec3 boxDim,NxReal density, int userData)
{
	for (int i = 0; i < stackDim.x; i++)
		for (int j = 0; j < stackDim.y; j++)
			for (int k = 0; k < stackDim.z; k++)
			{
				NxActor* actor = mActors->CreateBox(2*NxVec3(i*boxDim.x,j*boxDim.y,k*boxDim.z)+pos-0.5*NxVec3(stackDim.x*boxDim.x,0,stackDim.z*boxDim.z),boxDim,1.0);
				actor->userData = (void*)(userData+1);
			}
}

void Simulator::buildCastle()
{
	NxVec3 size(.2,.2,.2);
	myBuildStack(NxVec3(0,0,0),NxVec3(2,5,2),size,1.0,0);

	double x = 0.25, z=3;
	myBuildStack(NxVec3(-x,0,z),NxVec3(1,5,1),size,1.0,1);
	myBuildStack(NxVec3(x,0,-z),NxVec3(1,5,1),size,1.0,1);
	myBuildStack(NxVec3(z,0,x),NxVec3(1,5,1),size,1.0,1);
	myBuildStack(NxVec3(-z,0,-x),NxVec3(1,5,1),size,1.0,1);

	x = 7.5; z = 1; double y = 9.5;
	NxVec3 offset = 0.2*NxVec3(6,0,2);
	mActors->CreateBox(0.2*NxVec3(x,y,z),0.2*NxVec3(8,1,1),1.0);
	mActors->CreateBox(0.2*NxVec3(z,y,-x-1.0),0.2*NxVec3(1,1,8),1.0);
	mActors->CreateBox(0.2*NxVec3(-x-1.0,y,-z-1.0),0.2*NxVec3(8,1,1),1.0);
	mActors->CreateBox(0.2*NxVec3(-z-0.5,y,x-0.5),0.2*NxVec3(1,1,8),1.0);

	myBuildStack(0.2*NxVec3(0,10.55,0),NxVec3(2,5,2),size,1.0,2);

	threshActor = mActors->CreateBox(0.2*NxVec3(0,20.55,0),size*1.5,1.0);
}


void Simulator::ReleaseNx()
{
	if(mSDK != NULL)
	{
		if(mScene != NULL) mSDK->releaseScene(*mScene);
		mScene = NULL;
		NxReleasePhysicsSDK(mSDK);
		mSDK = NULL;
	}
	
	if (mAllocator)
	{
		delete mAllocator;
		mAllocator = NULL;
	}
}

void Simulator::Reset()
{
	ReleaseNx();
	win = false;
	if (!InitNx()) exit(0);
}


void Simulator::RunPhysics()
{
	// Update the time step
	deltaTime = getElapsedTime();
//	NxReal deltaTime = 0.0005;

	// Run collision and dynamics for delta time since the last frame
	if (!win)
	{
		mScene->simulate(deltaTime);	
		mScene->flushStream();
		mScene->fetchResults(NX_RIGID_BODY_FINISHED, true);
	}

	//check win
	if (threshActor==NULL) return;
	if (threshActor->getGlobalPosition().y<HEIGHT)
	{
		win = true;
	}
}

NxActor* Simulator::getSelectedActor() {
	return mActors->mSelectedActor;
}


Simulator::Simulator()
{
	mSDK = NULL;
	mScene = NULL;	
	mActors = NULL;
	mAllocator = NULL;
	
	mForceVec = NxVec3(0, 0, 0);
	mForceStrength = 10.0;
	mForceMode = true;
	mSelectedObject = NULL;
	win = false;
	levelCounter = 0;
}

Simulator::~Simulator()
{
	
	int nObject = mObjects.size();
	for(int i = 0; i < nObject; i++)
		mScene->releaseActor(*mObjects[i]);
	
	mObjects.clear(); 
}