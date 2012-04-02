/*
 *  Simulator.h
 *  BasicPhyx
 *
 *  Created by Karen Liu on 12/11/10.
 *  Copyright 2010 GA Tech. All rights reserved.
 *
 */

#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "NxPhysics.h"
#include <vector>

class Actors;
class UserAllocator;

class Simulator
{
public:
	NxPhysicsSDK *mSDK;
	NxScene *mScene;	
	NxActor *player;
	
private:
	Actors *mActors;
	UserAllocator *mAllocator;
	NxVec3 mForceVec;
	NxReal mForceStrength;
	bool mForceMode;
	NxReal deltaTime;
	static const int MAX_ARC_SEGS = 50;
	NxVec3 arc_segs[MAX_ARC_SEGS];
	int numSegs;
	NxVec3 PLANE[4];
	NxActor* threshActor;
	bool win;
	int levelCounter;
	
	
	std::vector<NxActor*> mObjects;
	NxActor *mSelectedObject;
	
public:
	Simulator();
	~Simulator();
	
	void buildCastle();
	void myBuildStack(NxVec3 pos,NxVec3 stackDim,NxVec3 boxDim,NxReal density,int userData);
	bool InitNx();
	void ReleaseNx();
	void CreateScene();
	void pressSpace(NxVec3 gCamForward);
	void holdSpace(NxVec3 gCamForward);
	void releaseSpace(NxVec3 gCamForward);
	
	void releaseActors();
	void nextLevel();
	void restartLevel();

	void RenderScene();
	void RunPhysics();
	void ProcessKeys(const bool *keys,NxVec3 gCamForward);

	NxActor* getSelectedActor();
		
private:
	void Reset();
	
	NxVec3 getPos(NxVec3 gCamForward);
	NxVec3 getLaunch(double t,NxVec3 gCamForward);
	void RenderStuff();
	void RenderActors();
	void RenderForce(NxActor* actor, NxVec3& forceVec, const NxVec3& color);	
	NxVec3 ApplyForceToActor(NxActor *actor, const NxVec3 & forceVec);
	
};

#endif