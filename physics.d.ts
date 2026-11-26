export const STATIC: number;
export const DYNAMIC: number;
export const KINEMATIC: number;
export const AWAKE: number;
export const SLEEPY: number;
export const SLEEPING: number;

export class Vec3 {
  constructor(x?: number, y?: number, z?: number);
  x: number;
  y: number;
  z: number;
  set(x?: number, y?: number, z?: number): this;
  copy(v: any): this;
  clone(): Vec3;
  [key: string]: any;
}

export class Quaternion {
  constructor(x?: number, y?: number, z?: number, w?: number);
  [key: string]: any;
}

export class Box {
  constructor(halfExtents?: any, y?: number, z?: number);
  halfExtents: Vec3;
  [key: string]: any;
}

export const BoxShape: typeof Box;
export const BoxCollider: typeof Box;

export class Body {
  static STATIC: number;
  static DYNAMIC: number;
  static KINEMATIC: number;
  static AWAKE: number;
  static SLEEPY: number;
  static SLEEPING: number;
  constructor(options?: any);
  id: number;
  position: Vec3;
  previousPosition: Vec3;
  velocity: Vec3;
  angularVelocity: Vec3;
  force: Vec3;
  mass: number;
  invMass: number;
  type: number | string;
  sleepState: number;
  allowSleep: boolean;
  sleepSpeedLimit: number;
  sleepTimeLimit: number;
  sleeping: boolean;
  asleep: boolean;
  readonly isSleeping: boolean;
  [key: string]: any;
  wakeUp(): this;
  sleep(): this;
  step?(dt: number): void;
}

export const RigidBody: typeof Body;
export const PhysicsBody: typeof Body;

export class World {
  constructor(options?: any);
  gravity: Vec3;
  bodies: Body[];
  solver: { iterations: number; [key: string]: any };
  allowSleep: boolean;
  iterations: number;
  solverIterations: number;
  [key: string]: any;
  addBody(...bodies: any[]): any;
  add(...bodies: any[]): this;
  removeBody(body: any): any;
  remove(...args: any[]): any;
  step(dt?: number, ...args: any[]): void;
  update(dt?: number, ...args: any[]): void;
  tick(dt?: number, ...args: any[]): void;
}

export const PhysicsWorld: typeof World;
export class Material { constructor(options?: any); [key: string]: any }
export class ContactMaterial { constructor(a?: any, b?: any, options?: any); [key: string]: any }
export class Solver { iterations: number; [key: string]: any }
export class NaiveBroadphase { [key: string]: any }

export function createVec3(x?: number, y?: number, z?: number): Vec3;
export function createBox(...args: any[]): Box;
export function createBody(options?: any): Body;
export function createWorld(options?: any): World;
export function createPhysics(options?: any): any;

declare const _default: any;
export default _default;
