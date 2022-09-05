// ammo_worker.js

// (c)2022 Simon Armstrong
// All rights reserved

// z is up

importScripts("ammo.min.js")

const Period=1000.0/60;

class _Box{
	constructor(w,h,d){
		this.size=new Ammo.btVector3(w,h,d);
		this.shape=new Ammo.btBoxShape(this.size);
	}
}

class _Sphere{
	constructor(radius){
		this.radius=radius;
		this.shape=new Ammo.btSphereShape(this.radius);
	}
}

class _Plane{
	constructor(nx,ny,nz,c){
		this.normal=new Ammo.btVector3(nx,ny,nz);
		this.plane=c;
		this.shape=new Ammo.btStaticPlaneShape(this.normal,c);
	}
}

/*
Uncaught ReferenceError ReferenceError: _emscripten_bind_btVector3_btVector3_1 is not defined
    at n (C:\nitrologic\roa\service\ammo.min.js:576:148)
*/

class _FloatTile{
	constructor(res,heights){
		const type="PHY_FLOAT";
		const scale=1.0/16;
		const axis=2;
		const data=Ammo._malloc(4*res*res);
		const ammoHeap=Ammo.HEAPF32;
		let min=1e6;
		let max=-1e6;
		for(let i=0;i<res*res;i++){
			const h=0;//heights[i];
			if(h<min) min=h;
			if(h>max) max=h;
			ammoHeap[data+i]=h;
		}
		this.shape=new Ammo.btHeightfieldTerrainShape(res,res,data, scale,min,max, axis,type,false);
	}
}

class _ShortTile{
	constructor(res,heights){
		const scale=1.0/16;
		const axis=2;
		const type="PHY_SHORT";
		const n=res*res;
		const data=Ammo._malloc(2*n);
		const ammoHeap=Ammo.HEAPI16;
		let min=1e6;
		let max=-1e6;
		for(let i=0;i<n;i++){
			const h=0;//heights[i];
			if(h<min) min=h;
			if(h>max) max=h;
			ammoHeap[data+i]=h;
		}
		this.shape=new Ammo.btHeightfieldTerrainShape(res,res,data, scale,min,max, axis,type,false);
	}
}


class _WorldBody{
	constructor(id,shape){
		this.shape=shape;
		this.rbInfo = new Ammo.btRigidBodyConstructionInfo(0, null, shape.shape, null);
		this.body = new Ammo.btRigidBody(this.rbInfo);
		this.body.userData=id;
	}
}

class _FixedBody{
	constructor(id,shape,x,y,z){
		this.shape=shape;
		const position=new Ammo.btVector3(x,y,z);
		const transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(position);
		this.motionState = new Ammo.btDefaultMotionState(transform);
		this.rbInfo = new Ammo.btRigidBodyConstructionInfo(0, this.motionState, shape.shape, null);
		this.body = new Ammo.btRigidBody(this.rbInfo);
		this.body.userData=id;
	}
}

class _DynamicBody{
	constructor(id,shape,mass,restitution,x,y,z){
		this.shape=shape;
		const inertia = new Ammo.btVector3(0, 0, 0);
		if(mass){
			shape.shape.calculateLocalInertia(mass,inertia);
		}
		const transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3(x,y,z));
		this.motionState = new Ammo.btDefaultMotionState(transform);
		this.rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, this.motionState, shape.shape, inertia);
		this.body = new Ammo.btRigidBody(this.rbInfo);
		this.body.setRestitution(restitution);
		this.body.userData=id;
	}
}

const MaxBodies=1e4;

class _World{
	playing=false;
	messageBuffer=[];
	bodies=[];
	dynamics=null;

	constructor(){
		self.onmessage = this.onmessage.bind(this);
	}

	// expect delay betwen constructor and reset, hence the messageBuffer

	contactAdded(){
	}
	
	contactProcessed(){
	}
	
	contactDestroyed(){
	}

	reset(ammo){
		this.tickCount=0;
		this.collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
		this.overlappingPairCache = new Ammo.btDbvtBroadphase();
		this.solver = new Ammo.btSequentialImpulseConstraintSolver();
		this.dispatcher = new Ammo.btCollisionDispatcher(this.collisionConfiguration);

		const dynamics=new Ammo.btDiscreteDynamicsWorld(this.dispatcher, this.overlappingPairCache, this.solver, this.collisionConfiguration);
		dynamics.setGravity(new Ammo.btVector3(0, 0, -10));

		dynamics.setContactAddedCallback(this.contactAdded.bind(this));
		dynamics.setContactProcessedCallback(this.contactAdded.bind(this));
		dynamics.setContactDestroyedCallback(this.contactDestroyed.bind(this));
			  
		this.dynamics=dynamics;

		this.process();
		this.run();
	}

	process(){
		for(const message of this.messageBuffer){
			this.onmessage(message);
		}
		this.messageBuffer=[];
	}

	post(commands,transfer){
		self.postMessage(commands,transfer);
	}

	execute(script){
		for(const line of script){
			const args=line.arguments;
			switch (line.command){
				case "play":
					this.playing=true;
					console.log("play");
					break;
				case "tile":{
					const id=args[0];
					const res=args[1];
					const heights=args[2];
					const x=args[3];
					const y=args[4];
					const tile=new _FloatTile(res,heights);
					const body=new _FixedBody(id,tile,x,y,0);
					this.bodies[id]=body;
					this.dynamics.addRigidBody(body.body);
					console.log("tile");
				}break;
				case "plane":{
					const id=args[0];
					const nx=args[1];
					const ny=args[2];
					const nz=args[3];
					const nc=args[4];
					const plane=new _Plane(nx,ny,nz,nc);
					const body=new _WorldBody(id,plane);
					this.bodies[id]=body;
					this.dynamics.addRigidBody(body.body);
					console.log("plane");
				}break;
				case "ball":{
					const id=args[0];
					const mass=args[1];
					const restitution=args[2];
					const radius=args[3];
					const x=args[4];
					const y=args[5];
					const z=args[6];
					const sphere=new _Sphere(radius);
					const body=new _DynamicBody(id,sphere,mass,restitution,x,y,z);
					this.bodies[id]=body;
					this.dynamics.addRigidBody(body.body);
					console.log("ball:",x,y,z);
				}break;
				case "box":{
					const id=args[0];
					const mass=args[1];
					const restitution=args[2];
					const w=args[3];
					const h=args[4];
					const d=args[5];
					const x=args[6];
					const y=args[7];
					const z=args[8];
					const box=new _Box(w,h,d);
					const body=new _DynamicBody(id,box,mass,restitution,x,y,z);
					this.bodies[id]=body;
					this.dynamics.addRigidBody(body.body);
					console.log("box");
				}break;
			}
		}
	}

	play(){
		console.log("play");
	}

	run(){	
		console.log("ammo_worker run period:"+Period);
		if (this.interval){ 
			clearInterval(this.interval);
		}
		this.tickCount=0;
		const stepthis=this.stepAmmo.bind(this);
		this.interval=setInterval(stepthis,Period);
		this.post([{command:"running"}]);
	}

	stepAmmo(){
		if(!this.dynamics) return;
//		console.log('.');
		const count=this.tickCount++;
		const timestep=count*Period/1000;
		this.dynamics.stepSimulation(timestep, 2);

		const activeBodies=[];

		for(const id in this.bodies){
			const body=this.bodies[id];
			if(body.body.isActive()){
				activeBodies.push(body.body);
			}
		}

		const n=activeBodies.length;
		const raw=new Float64Array(n*8);

		const active=[];
		let p=0;

		for(const body of activeBodies){
			const id=body.userData;
			active.push(id);

			const transform=body.getWorldTransform();
			const position = transform.getOrigin();
			const rotation = transform.getRotation();

			raw[p+0] = position.x();
			raw[p+1] = position.y();
			raw[p+2] = position.z();
			raw[p+3] = 0.0;

			raw[p+4] = rotation.x();
			raw[p+5] = rotation.y();
			raw[p+6] = rotation.z();
			raw[p+7] = rotation.w();

			p+=8;
		}

		this.post([{command:"tick",count,active,array:raw}],[raw.buffer]);
	}

	onmessage(e){
		if(!this.dynamics){
			this.messageBuffer.push(e);
		}else{
			const script=e.data;
			this.execute(script);
		}
	}
}

const _world=new _World();

Ammo().then(ammo=>{
	console.log("Bullet 2.82 Physics (c)2013 Erwin Coumans");
	_world.reset(ammo);
});

