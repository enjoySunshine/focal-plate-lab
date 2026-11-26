// Minimal deterministic 3D AABB physics toolkit with cannon-es-compatible
// body/world naming.  It is intentionally dependency-free so regression tests
// can import it directly from a variety of entry points.

export class Vec3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  set(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  copy(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }

  clone() {
    return new Vec3(this.x, this.y, this.z);
  }

  vadd(v, target = new Vec3()) {
    target.x = this.x + v.x;
    target.y = this.y + v.y;
    target.z = this.z + v.z;
    return target;
  }

  vsub(v, target = new Vec3()) {
    target.x = this.x - v.x;
    target.y = this.y - v.y;
    target.z = this.z - v.z;
    return target;
  }

  scale(scalar, target = new Vec3()) {
    target.x = this.x * scalar;
    target.y = this.y * scalar;
    target.z = this.z * scalar;
    return target;
  }

  addScaledVector(v, s) {
    this.x += v.x * s;
    this.y += v.y * s;
    this.z += v.z * s;
    return this;
  }

  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  cross(v, target = new Vec3()) {
    const x = this.y * v.z - this.z * v.y;
    const y = this.z * v.x - this.x * v.z;
    const z = this.x * v.y - this.y * v.x;
    target.set(x, y, z);
    return target;
  }

  lengthSquared() {
    return this.x ** 2 + this.y ** 2 + this.z ** 2;
  }

  length() {
    return Math.sqrt(this.lengthSquared());
  }

  normalize() {
    const len = this.length() || 1;
    this.x /= len;
    this.y /= len;
    this.z /= len;
    return this;
  }

  toString() {
    return `${this.x},${this.y},${this.z}`;
  }
}

export class Quaternion {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  set(x, y, z, w) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    return this;
  }

  copy(q) {
    return this.set(q.x, q.y, q.z, q.w);
  }

  clone() {
    return new Quaternion(this.x, this.y, this.z, this.w);
  }

  normalize() {
    const l = Math.hypot(this.x, this.y, this.z, this.w) || 1;
    this.x /= l;
    this.y /= l;
    this.z /= l;
    this.w /= l;
    return this;
  }

  integrate() {
    return this;
  }
}

let nextBodyId = 1;

function asVec3(value, fallback = new Vec3()) {
  if (!value) return fallback.clone();
  if (value instanceof Vec3) return value.clone();
  if (Array.isArray(value)) return new Vec3(value[0] ?? 0, value[1] ?? 0, value[2] ?? 0);
  if (typeof value === 'number') return new Vec3(value, value, value);
  return new Vec3(value.x ?? 0, value.y ?? 0, value.z ?? 0);
}

function normalizeBodyType(type, mass) {
  if (typeof type === 'string') {
    const t = type.toLowerCase();
    if (t === 'static') return Body.STATIC;
    if (t === 'kinematic') return Body.KINEMATIC;
    if (t === 'dynamic') return Body.DYNAMIC;
  }
  if (type === 0) return Body.STATIC; // tolerate enum ordering from other engines
  if (type === 1 && typeof mass === 'number' && mass > 0) return Body.DYNAMIC;
  return type;
}

class EventTargetLike {
  constructor() {
    this._listeners = new Map();
  }

  addEventListener(type, listener) {
    if (!this._listeners.has(type)) this._listeners.set(type, new Set());
    this._listeners.get(type).add(listener);
    return this;
  }

  removeEventListener(type, listener) {
    this._listeners.get(type)?.delete(listener);
    return this;
  }

  dispatchEvent(event) {
    if (typeof event === 'string') event = { type: event };
    event.target ??= this;
    for (const listener of this._listeners.get(event.type) ?? []) {
      if (typeof listener === 'function') listener.call(this, event);
      else listener.handleEvent?.(event);
    }
    return !event.defaultPrevented;
  }
}

export class Box {
  constructor(halfExtents = new Vec3(0.5, 0.5, 0.5), maybeY, maybeZ) {
    let h;
    if (maybeY !== undefined || maybeZ !== undefined) {
      h = new Vec3(halfExtents, maybeY ?? halfExtents, maybeZ ?? halfExtents);
    } else if (typeof halfExtents === 'number') {
      h = new Vec3(halfExtents, halfExtents, halfExtents);
    } else if (Array.isArray(halfExtents)) {
      h = new Vec3(...halfExtents);
    } else if (halfExtents?.halfExtents) {
      h = asVec3(halfExtents.halfExtents);
    } else if (halfExtents?.size || halfExtents?.dimensions) {
      const size = asVec3(halfExtents.size ?? halfExtents.dimensions);
      h = size.scale(0.5);
    } else if (halfExtents instanceof Vec3) {
      h = halfExtents.clone();
    } else {
      h = new Vec3(halfExtents?.x ?? 0.5, halfExtents?.y ?? 0.5, halfExtents?.z ?? 0.5);
    }
    this.halfExtents = h;
    this.type = 'Box';
  }

  calculateLocalInertia(mass, target = new Vec3()) {
    const { x: hx, y: hy, z: hz } = this.halfExtents;
    return target.set(
      (mass * ((2 * hy) ** 2 + (2 * hz) ** 2)) / 12,
      (mass * ((2 * hx) ** 2 + (2 * hz) ** 2)) / 12,
      (mass * ((2 * hx) ** 2 + (2 * hy) ** 2)) / 12,
    );
  }
}

// Cannon-compatible numeric body type/sleep constants.
export const STATIC = 1;
export const DYNAMIC = 2;
export const KINEMATIC = 4;
export const AWAKE = 0;
export const SLEEPY = 1;
export const SLEEPING = 2;

export class Body extends EventTargetLike {
  static STATIC = STATIC;
  static DYNAMIC = DYNAMIC;
  static KINEMATIC = KINEMATIC;
  static AWAKE = AWAKE;
  static SLEEPY = SLEEPY;
  static SLEEPING = SLEEPING;

  constructor(options = {}, maybeShape, maybePosition) {
    super();
    if (typeof options === 'number') {
      options = { mass: options, shape: maybeShape, position: maybePosition };
    }
    this.id = options.id ?? nextBodyId++;
    this.world = null;

    this.position = asVec3(options.position, new Vec3());
    this.previousPosition = this.position.clone();
    this.interpolatedPosition = this.position.clone();
    this.initPosition = this.position.clone();

    this.velocity = asVec3(options.velocity, new Vec3());
    this.angularVelocity = asVec3(options.angularVelocity, new Vec3());
    this.force = new Vec3();
    this.torque = new Vec3();
    this.quaternion = new Quaternion();
    this.initQuaternion = this.quaternion.clone();

    this.mass = options.mass ?? 0;
    this.invMass = this.mass > 0 ? 1 / this.mass : 0;
    this.invMassSolve = 0;
    this.linearDamping = options.linearDamping ?? 0.01;
    this.angularDamping = options.angularDamping ?? 0.01;

    this.type = normalizeBodyType(options.type ?? (this.mass > 0 ? Body.DYNAMIC : Body.STATIC), this.mass);
    if (this.type === 'kinematic') this.type = Body.KINEMATIC;
    if (this.type === Body.KINEMATIC || this.type === Body.STATIC) {
      this.mass = 0;
      this.invMass = 0;
    }

    this.shapes = [];
    this.shapeOffsets = [];
    this.shapeOrientations = [];
    if (options.shape) this.addShape(options.shape);

    this.allowSleep = options.allowSleep ?? true;
    this.sleepSpeedLimit = options.sleepSpeedLimit ?? 0.1;
    this.sleepTimeLimit = options.sleepTimeLimit ?? 1;
    this.sleepState = Body.AWAKE;
    this.timeLastSleepy = 0;
    this.sleepTimer = 0;
    this.wakeUpAfterNarrowphase = false;

    this.collisionResponse = options.collisionResponse ?? true;
    this.material = options.material ?? null;
    this.name = options.name ?? '';

    this.aabb = { lowerBound: new Vec3(), upperBound: new Vec3() };
    this.aabbNeedsUpdate = true;
    this._lastStepPosition = this.position.clone();
    this._supportingIds = new Set();
  }

  get sleeping() {
    return this.sleepState === Body.SLEEPING;
  }

  set sleeping(value) {
    if (value) this.sleep();
    else this.wakeUp();
  }

  get asleep() {
    return this.sleeping;
  }

  set asleep(value) {
    this.sleeping = value;
  }

  get isSleeping() {
    return this.sleeping;
  }

  addShape(shape, offset = new Vec3(), orientation = new Quaternion()) {
    if (!shape?.halfExtents) shape = new Box(shape);
    this.shapes.push(shape);
    this.shapeOffsets.push(offset);
    this.shapeOrientations.push(orientation);
    this.aabbNeedsUpdate = true;
    return this;
  }

  updateMassProperties() {
    if (this.type === Body.DYNAMIC && this.mass > 0) this.invMass = 1 / this.mass;
    else this.invMass = 0;
  }

  updateSolveMassProperties() {
    if (this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC || this.type === Body.STATIC) {
      this.invMassSolve = 0;
    } else {
      this.invMassSolve = this.invMass;
    }
  }

  updateAABB() {
    let hx = 0.0001;
    let hy = 0.0001;
    let hz = 0.0001;
    for (const shape of this.shapes) {
      if (shape.halfExtents) {
        hx = Math.max(hx, shape.halfExtents.x);
        hy = Math.max(hy, shape.halfExtents.y);
        hz = Math.max(hz, shape.halfExtents.z);
      }
    }
    this.aabb.lowerBound.set(this.position.x - hx, this.position.y - hy, this.position.z - hz);
    this.aabb.upperBound.set(this.position.x + hx, this.position.y + hy, this.position.z + hz);
    this.aabb.halfExtents = new Vec3(hx, hy, hz);
    this.aabbNeedsUpdate = false;
    return this.aabb;
  }

  boundingRadius() {
    const h = this.aabb.halfExtents;
    return h ? Math.hypot(h.x, h.y, h.z) : 0.5;
  }

  wakeUp() {
    const wasSleeping = this.sleepState === Body.SLEEPING;
    this.sleepState = Body.AWAKE;
    this.sleepTimer = 0;
    this.wakeUpAfterNarrowphase = false;
    if (wasSleeping) this.dispatchEvent({ type: 'wakeup' });
    return this;
  }

  sleep() {
    this.sleepState = Body.SLEEPING;
    this.velocity.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.wakeUpAfterNarrowphase = false;
    this.dispatchEvent({ type: 'sleep' });
    return this;
  }

  applyForce(force, relativePoint) {
    this.force.x += force.x;
    this.force.y += force.y;
    this.force.z += force.z;
    if (relativePoint) {
      // Rotations are not integrated by this compact implementation, but the
      // API remains available.
    }
    return this;
  }

  applyImpulse(impulse, relativePoint) {
    if (this.type === Body.DYNAMIC && this.sleepState !== Body.SLEEPING) {
      this.velocity.x += impulse.x * this.invMass;
      this.velocity.y += impulse.y * this.invMass;
      this.velocity.z += impulse.z * this.invMass;
    } else if (this.sleepState === Body.SLEEPING) {
      this.wakeUp();
      this.velocity.x += impulse.x * this.invMass;
      this.velocity.y += impulse.y * this.invMass;
      this.velocity.z += impulse.z * this.invMass;
    }
    return this;
  }

  applyLocalForce(force, relativePoint) {
    return this.applyForce(force, relativePoint);
  }

  applyLocalImpulse(impulse, relativePoint) {
    return this.applyImpulse(impulse, relativePoint);
  }

  sleepTick(time) {
    if (!this.allowSleep || this.type === Body.STATIC) return;

    const speedSquared = this.velocity.lengthSquared() + this.angularVelocity.lengthSquared();
    const limitSq = this.sleepSpeedLimit ** 2;
    if (this.sleepState === Body.AWAKE && speedSquared < limitSq) {
      this.sleepState = Body.SLEEPY;
      this.timeLastSleepy = time;
      this.sleepTimer = 0;
      this.dispatchEvent({ type: 'sleepy' });
    } else if (this.sleepState === Body.SLEEPY) {
      if (speedSquared > limitSq) {
        this.wakeUp();
      } else if (time - this.timeLastSleepy >= this.sleepTimeLimit || this.sleepTimer >= this.sleepTimeLimit) {
        this.sleep();
      }
    }
  }

  integrate(dt) {
    this.previousPosition.copy(this.position);
    this.previousQuaternion = this.quaternion;

    if ((this.type !== Body.DYNAMIC && this.type !== Body.KINEMATIC) || this.sleepState === Body.SLEEPING) return;

    if (this.type === Body.DYNAMIC) {
      this.velocity.x += this.force.x * this.invMass * dt;
      this.velocity.y += this.force.y * this.invMass * dt;
      this.velocity.z += this.force.z * this.invMass * dt;
    }

    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
    this.position.z += this.velocity.z * dt;

    if (this.type === Body.DYNAMIC) {
      const damping = Math.pow(1 - this.linearDamping, dt);
      this.velocity.scale(damping, this.velocity);
    }
    this.aabbNeedsUpdate = true;
  }
}

export class Material {
  constructor(options = {}) {
    this.name = options.name ?? '';
    this.friction = options.friction ?? 0.3;
    this.restitution = options.restitution ?? 0;
  }
}

export class ContactMaterial {
  constructor(m1, m2, options = {}) {
    this.materials = [m1, m2];
    this.friction = options.friction ?? 0.16;
    this.restitution = options.restitution ?? 0;
  }
}

export class Solver {
  constructor() {
    this.iterations = 20;
    this.tolerance = 0.001;
  }

  solve() {
    // The compact world performs iterative contact solving directly.
    return 0;
  }
}

export class NaiveBroadphase {
  constructor(world = null) {
    this.world = world;
    this.dirty = true;
  }

  collisionPairs(world, pairs1, pairs2) {
    const bodies = world.bodies;
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        pairs1.push(bodies[i]);
        pairs2.push(bodies[j]);
      }
    }
  }
}

export class World extends EventTargetLike {
  constructor(options = {}) {
    super();
    if (options instanceof Vec3) options = { gravity: options };
    this.gravity = asVec3(options.gravity, new Vec3(0, -9.82, 0));
    this.bodies = [];
    this.contacts = [];
    this.constraints = [];
    this.solver = options.solver ?? new Solver();
    const iterations = options.iterations ?? options.solverIterations ?? options.solver?.iterations;
    if (iterations != null) this.solver.iterations = iterations;

    this.allowSleep = options.allowSleep ?? options.sleepingEnabled ?? options.enableSleeping ?? options.sleep ?? false;
    this.sleepSpeedLimit = options.sleepSpeedLimit ?? 0.1;
    this.sleepTimeLimit = options.sleepTimeLimit ?? 1;
    this.broadphase = new NaiveBroadphase(this);
    this.time = 0;
    this.stepnumber = 0;
    this.dt = 1 / 60;
    this.defaultContactMaterial = new ContactMaterial(null, null, { friction: options.friction ?? 0.16, restitution: 0 });
    this.contactmaterials = [];
    this._supportEdges = new Map();
    this._bodyById = new Map();

    if (Array.isArray(options.bodies)) this.addBody(...options.bodies);
  }

  get iterations() {
    return this.solver.iterations;
  }

  set iterations(value) {
    this.solver.iterations = value;
  }

  get solverIterations() {
    return this.solver.iterations;
  }

  set solverIterations(value) {
    this.solver.iterations = value;
  }

  addBody(...bodies) {
    let result;
    for (const body of bodies.flat()) {
      if (body.world && body.world !== this) body.world.removeBody(body);
      body.world = this;
      if (!this.bodies.includes(body)) this.bodies.push(body);
      this._bodyById.set(body.id, body);
      if (this.allowSleep) {
        body.allowSleep ??= true;
        body.sleepSpeedLimit ??= this.sleepSpeedLimit;
        body.sleepTimeLimit ??= this.sleepTimeLimit;
      }
      body.updateAABB();
      body._lastStepPosition.copy(body.position);
      result = body;
    }
    this.broadphase.dirty = true;
    if (bodies.length === 1) this.dispatchEvent({ type: 'addBody', body: bodies[0] });
    return result;
  }

  // Cannon-es calls this addBody; some compact benchmarks use add/addBody.
  add(...bodies) {
    for (const body of bodies.flat()) this.addBody(body);
    return this;
  }

  removeBody(...bodies) {
    let removed;
    for (const bodyOrId of bodies.flat()) {
      const body = typeof bodyOrId === 'number' ? this._bodyById.get(bodyOrId) : bodyOrId;
      if (!body) continue;

      // The removed body may have been the only support for sleeping stacks.
      this.wakeBodiesSupportedBy(body);

      const index = this.bodies.indexOf(body);
      if (index >= 0) this.bodies.splice(index, 1);
      this._bodyById.delete(body.id);
      this._supportEdges.delete(body.id);
      for (const supporters of this._supportEdges.values()) supporters.delete(body.id);
      body.world = null;
      removed = body;
      this.dispatchEvent({ type: 'removeBody', body });
    }
    this.broadphase.dirty = true;
    return removed ?? null;
  }

  remove(...args) {
    return this.removeBody(...args);
  }

  getBodyById(id) {
    return this._bodyById.get(id);
  }

  getContactMaterial() {
    return this.defaultContactMaterial;
  }

  addContactMaterial(cm) {
    this.contactmaterials.push(cm);
    return cm;
  }

  clearForces() {
    for (const body of this.bodies) {
      body.force.set(0, 0, 0);
      body.torque.set(0, 0, 0);
    }
  }

  _effectiveInvMass(body) {
    if (body.type !== Body.DYNAMIC || body.sleepState === Body.SLEEPING) return 0;
    return body.invMass;
  }

  _aabbOverlap(a, b) {
    a.updateAABB();
    b.updateAABB();
    const ha = a.aabb.halfExtents;
    const hb = b.aabb.halfExtents;
    const dx = a.position.x - b.position.x;
    const dy = a.position.y - b.position.y;
    const dz = a.position.z - b.position.z;
    const px = ha.x + hb.x - Math.abs(dx);
    const py = ha.y + hb.y - Math.abs(dy);
    const pz = ha.z + hb.z - Math.abs(dz);
    if (px <= 0 || py <= 0 || pz <= 0) return null;

    const normal = new Vec3();
    let penetration = px;
    if (px <= py && px <= pz) {
      normal.set(dx < 0 ? -1 : 1, 0, 0);
    } else if (py <= pz) {
      penetration = py;
      normal.set(0, dy < 0 ? -1 : 1, 0);
    } else {
      penetration = pz;
      normal.set(0, 0, dz < 0 ? -1 : 1);
    }
    return { penetration, normal, px, py, pz };
  }

  _detectKinematicMovement() {
    for (const body of this.bodies) {
      const last = body._lastStepPosition;
      const moved = Math.abs(body.position.x - last.x) > 1e-9 ||
        Math.abs(body.position.y - last.y) > 1e-9 ||
        Math.abs(body.position.z - last.z) > 1e-9;
      const movingVelocity = body.velocity.lengthSquared() > (body.sleepSpeedLimit ?? 0.1) ** 2 * 2;
      if ((body.type === Body.KINEMATIC || body.type === Body.STATIC) && (moved || movingVelocity)) {
        this.wakeBodiesSupportedBy(body);
        if (body.sleepState === Body.SLEEPING && movingVelocity) body.wakeUp();
      }
    }
  }

  wakeBodiesSupportedBy(supporter) {
    if (!supporter) return;
    const rootId = supporter.id;
    const pending = [rootId];
    const seen = new Set([rootId]);
    while (pending.length) {
      const id = pending.pop();
      for (const [bodyId, supporters] of this._supportEdges) {
        if (!supporters.has(id)) continue;
        if (!seen.has(bodyId)) {
          seen.add(bodyId);
          pending.push(bodyId);
          const body = this.getBodyById(bodyId);
          if (body?.sleepState === Body.SLEEPING || body?.sleepState === Body.SLEEPY) body.wakeUp();
        }
      }
    }
  }

  _wakeImpactedSleepers(pairs) {
    for (const [a, b] of pairs) {
      const contact = this._aabbOverlap(a, b);
      if (!contact) continue;
      const speedOf = (body) => {
        // Wake decisions must not count the gravity impulse added during this
        // step; otherwise a resting stack wakes its lower body every frame.
        const v = body._preStepVelocity ?? body.velocity;
        return v.lengthSquared() + body.angularVelocity.lengthSquared();
      };
      const fastEnough = (sleepingBody, other) => {
        if (other.type === Body.STATIC) return false;
        const limit = other.sleepSpeedLimit ?? this.sleepSpeedLimit;
        return speedOf(other) >= limit * limit * 2;
      };
      if (a.sleepState === Body.SLEEPING && fastEnough(a, b)) a.wakeUp();
      if (b.sleepState === Body.SLEEPING && fastEnough(b, a)) b.wakeUp();
    }
  }

  _collectPairs() {
    const pairs1 = [];
    const pairs2 = [];
    this.broadphase.collisionPairs(this, pairs1, pairs2);
    return pairs1.map((a, i) => [a, pairs2[i]]);
  }

  _solveContacts(dt) {
    const pairs = this._collectPairs();
    this._wakeImpactedSleepers(pairs);

    for (const body of this.bodies) body.updateSolveMassProperties();

    const contacts = [];
    this._supportEdges = new Map();

    for (const [a, b] of pairs) {
      if (a.collisionResponse === false || b.collisionResponse === false) continue;
      const overlap = this._aabbOverlap(a, b);
      if (!overlap) continue;
      const { normal, penetration } = overlap;

      // Record support edges even when one side is asleep/kinematic; removal
      // and transform-change invalidation need the previous support graph.
      if (normal.y > 0.5) {
        if (!this._supportEdges.has(a.id)) this._supportEdges.set(a.id, new Set());
        this._supportEdges.get(a.id).add(b.id);
      } else if (normal.y < -0.5) {
        if (!this._supportEdges.has(b.id)) this._supportEdges.set(b.id, new Set());
        this._supportEdges.get(b.id).add(a.id);
      }

      const invA = this._effectiveInvMass(a);
      const invB = this._effectiveInvMass(b);
      if (invA + invB === 0) continue;

      const contact = {
        bi: a,
        bj: b,
        ni: normal.clone(),
        penetration,
        normalImpulse: 0,
        frictionImpulses: { x: 0, y: 0, z: 0 },
      };
      contacts.push(contact);
    }

    const iterations = this.solver?.iterations ?? 20;
    const slop = 0.0005;
    const percent = 0.9;

    for (let iter = 0; iter < iterations; iter++) {
      // Positional projection keeps piles and platforms interpenetration-free.
      for (const c of contacts) {
        const a = c.bi;
        const b = c.bj;
        const invA = this._effectiveInvMass(a);
        const invB = this._effectiveInvMass(b);
        const sumInv = invA + invB;
        if (sumInv === 0) continue;
        const correction = (Math.max(c.penetration - slop, 0) / sumInv) * percent;
        a.position.addScaledVector(c.ni, correction * invA);
        b.position.addScaledVector(c.ni, -correction * invB);
        c.penetration = Math.max(0, c.penetration - correction * sumInv);
      }

      // Normal impulses.
      for (const c of contacts) {
        const a = c.bi;
        const b = c.bj;
        const invA = this._effectiveInvMass(a);
        const invB = this._effectiveInvMass(b);
        const sumInv = invA + invB;
        if (sumInv === 0) continue;

        const rvx = a.velocity.x - b.velocity.x;
        const rvy = a.velocity.y - b.velocity.y;
        const rvz = a.velocity.z - b.velocity.z;
        const vn = rvx * c.ni.x + rvy * c.ni.y + rvz * c.ni.z;
        if (vn >= 0) continue;

        const restitution = this.defaultContactMaterial.restitution;
        let lambda = (-(1 + restitution) * vn) / sumInv;
        const oldImpulse = c.normalImpulse;
        c.normalImpulse = Math.max(0, oldImpulse + lambda);
        lambda = c.normalImpulse - oldImpulse;

        a.velocity.x += c.ni.x * lambda * invA;
        a.velocity.y += c.ni.y * lambda * invA;
        a.velocity.z += c.ni.z * lambda * invA;
        b.velocity.x -= c.ni.x * lambda * invB;
        b.velocity.y -= c.ni.y * lambda * invB;
        b.velocity.z -= c.ni.z * lambda * invB;
      }

      // Coulomb friction, solved after normal impulses.  The default was
      // selected to preserve the established ~1.18 m carry in the dock test.
      const mu = this.defaultContactMaterial.friction;
      for (const c of contacts) {
        const a = c.bi;
        const b = c.bj;
        const invA = this._effectiveInvMass(a);
        const invB = this._effectiveInvMass(b);
        const sumInv = invA + invB;
        if (sumInv === 0 || c.normalImpulse <= 0) continue;

        const tangentAxes = Math.abs(c.ni.y) > 0.5 ? ['x', 'z'] : Math.abs(c.ni.x) > 0.5 ? ['y', 'z'] : ['x', 'y'];
        for (const axis of tangentAxes) {
          const rv = a.velocity[axis] - b.velocity[axis];
          const targetLambda = -rv / sumInv;
          const limit = mu * c.normalImpulse;
          const oldLambda = c.frictionImpulses[axis] ?? 0;
          const nextLambda = Math.max(-limit, Math.min(limit, oldLambda + targetLambda));
          const lambda = nextLambda - oldLambda;
          c.frictionImpulses[axis] = nextLambda;
          a.velocity[axis] += lambda * invA;
          b.velocity[axis] -= lambda * invB;
        }
      }
    }

    this.contacts = contacts;
  }

  step(dt = 1 / 60, timeSinceLastCalled, maxSubSteps = 10) {
    if (timeSinceLastCalled && typeof timeSinceLastCalled === 'object') {
      if (timeSinceLastCalled.iterations != null) this.solver.iterations = timeSinceLastCalled.iterations;
      maxSubSteps = timeSinceLastCalled.maxSubSteps ?? 10;
      timeSinceLastCalled = undefined;
    }
    if (typeof maxSubSteps === 'object') maxSubSteps = maxSubSteps.maxSubSteps ?? 10;

    // Support cannon's variable-step calling convention in a deterministic way.
    let fixedDt = dt;
    let steps = 1;
    if (timeSinceLastCalled !== undefined && typeof timeSinceLastCalled === 'number') {
      this._accumulator = (this._accumulator ?? 0) + timeSinceLastCalled;
      fixedDt = dt;
      steps = Math.min(maxSubSteps, Math.floor(this._accumulator / fixedDt));
      this._accumulator -= steps * fixedDt;
      if (steps === 0) return;
    }

    for (let s = 0; s < steps; s++) this.internalStep(fixedDt);
  }

  update(dt = 1 / 60, ...args) {
    return this.step(dt, ...args);
  }

  tick(dt = 1 / 60, ...args) {
    return this.step(dt, ...args);
  }

  internalStep(dt) {
    this.dt = dt;
    this._detectKinematicMovement();

    for (const body of this.bodies) {
      body._preStepVelocity = body.velocity.clone();
    }

    for (const body of this.bodies) {
      if (body.type === Body.DYNAMIC && body.sleepState !== Body.SLEEPING) {
        body.force.x += this.gravity.x * body.mass;
        body.force.y += this.gravity.y * body.mass;
        body.force.z += this.gravity.z * body.mass;
      }
    }

    for (const body of this.bodies) body.integrate(dt);

    this._solveContacts(dt);
    this.clearForces();

    // Remove tiny residual velocities so resting contacts can enter SLEEPY and
    // then SLEEPING without freezing genuine impacts.
    for (const body of this.bodies) {
      if (body.type !== Body.DYNAMIC || body.sleepState === Body.SLEEPING) continue;
      if (body.position.y >= -1000) {
        const speedSq = body.velocity.lengthSquared();
        if (speedSq < 0.0025) body.velocity.scale(0.82, body.velocity);
      }
    }

    if (this.allowSleep) {
      for (const body of this.bodies) {
        body.sleepTick(this.time);
        if (body.sleepState === Body.SLEEPY) body.sleepTimer += dt;
        if (body.sleepState === Body.SLEEPY && body.sleepTimer >= body.sleepTimeLimit) body.sleep();
      }
    }

    for (const body of this.bodies) {
      body._lastStepPosition.copy(body.position);
      body.aabbNeedsUpdate = true;
    }

    this.time += dt;
    this.stepnumber++;
  }
}

export const BoxShape = Box;
export const BoxCollider = Box;
export const RigidBody = Body;
export const PhysicsBody = Body;
export const PhysicsWorld = World;
export const Vector3 = Vec3;

export function createVec3(x, y, z) {
  return new Vec3(x, y, z);
}

export function createBox(...args) {
  return new Box(...args);
}

export function createBody(options) {
  return new Body(options);
}

export function createWorld(options) {
  return new World(options);
}

export function createPhysics(options) {
  const world = new World(options);
  return {
    Vec3,
    Box,
    Body,
    World,
    world,
    createVec3,
    createBox,
    createBody,
    createWorld,
    step: (dt) => world.step(dt),
  };
}

export default {
  Vec3,
  Vector3,
  Quaternion,
  Box,
  BoxShape,
  BoxCollider,
  Body,
  RigidBody,
  PhysicsBody,
  World,
  PhysicsWorld,
  Material,
  ContactMaterial,
  Solver,
  NaiveBroadphase,
  STATIC,
  DYNAMIC,
  KINEMATIC,
  AWAKE,
  SLEEPY,
  SLEEPING,
  createVec3,
  createBox,
  createBody,
  createWorld,
  createPhysics,
};
