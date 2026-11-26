import test from 'node:test';
import assert from 'node:assert/strict';
import { World, Body, Box, Vec3 } from './physics.js';

function makeWorld() {
	const world = new World({ gravity: new Vec3(0, -10, 0), allowSleep: true });
	world.solver.iterations = 20;
	world.sleepSpeedLimit = 0.1;
	world.sleepTimeLimit = 1;
	return world;
}

function crate() {
	return new Body({
		mass: 1,
		shape: new Box(new Vec3(0.3, 0.3, 0.3)),
		sleepSpeedLimit: 0.1,
		sleepTimeLimit: 1,
	});
}

function platform(halfX, type = Body.STATIC) {
	return new Body({ mass: 0, type, shape: new Box(new Vec3(halfX, 0.2, 2)) });
}

test('resting crate sleeps at supported height', () => {
	const world = makeWorld();
	const ground = platform(2);
	const box = crate();
	box.position.set(0, 1, 0);
	world.addBody(ground, box);

	for (let i = 0; i < 150; i++) world.step(1 / 60);
	assert.ok(Math.abs(box.position.y - 0.5) <= 0.01);
	assert.equal(box.sleepState, Body.SLEEPING);

	for (let i = 0; i < 180; i++) world.step(1 / 60);
	assert.ok(Math.abs(box.position.y - 0.5) <= 0.01);
	assert.equal(box.sleepState, Body.SLEEPING);
});

test('sleeping stack preserves both heights', () => {
	const world = makeWorld();
	const ground = platform(2);
	const lower = crate();
	const upper = crate();
	lower.position.set(0, 1, 0);
	upper.position.set(0, 2, 0);
	world.addBody(ground, lower, upper);

	for (let i = 0; i < 240; i++) world.step(1 / 60);
	assert.ok(Math.abs(lower.position.y - 0.5) <= 0.01);
	assert.ok(Math.abs(upper.position.y - 1.1) <= 0.01);
	assert.equal(lower.sleepState, Body.SLEEPING);
	assert.equal(upper.sleepState, Body.SLEEPING);
});

test('raised kinematic elevator carries sleeping crate', () => {
	const world = makeWorld();
	const elevator = platform(2, Body.KINEMATIC);
	const box = crate();
	box.position.set(0, 1, 0);
	world.addBody(elevator, box);

	for (let i = 0; i < 150; i++) world.step(1 / 60);
	assert.equal(box.sleepState, Body.SLEEPING);

	for (let i = 0; i < 60; i++) {
		elevator.position.y += 0.02;
		world.step(1 / 60);
	}

	assert.ok(box.position.y > 1.6 && box.position.y < 1.8);
	assert.notEqual(box.sleepState, Body.SLEEPING);
});

test('driven-away barge lets unsupported sleeping crate fall', () => {
	const world = makeWorld();
	const barge = platform(1, Body.KINEMATIC);
	const box = crate();
	box.position.set(0, 1, 0);
	world.addBody(barge, box);

	for (let i = 0; i < 150; i++) world.step(1 / 60);
	for (let i = 0; i < 80; i++) {
		barge.position.x += 0.05;
		world.step(1 / 60);
	}

	assert.ok(box.position.y < -1);
});

test('removed support wakes sleeping crate immediately', () => {
	const world = makeWorld();
	const ground = platform(2);
	const box = crate();
	box.position.set(0, 1, 0);
	world.addBody(ground, box);

	for (let i = 0; i < 150; i++) world.step(1 / 60);
	world.removeBody(ground);
	assert.notEqual(box.sleepState, Body.SLEEPING);

	for (let i = 0; i < 90; i++) world.step(1 / 60);
	assert.ok(box.position.y < -10);
});

test('moving elevator preserves established horizontal carry', () => {
	const world = makeWorld();
	const elevator = platform(2, Body.KINEMATIC);
	const box = crate();
	box.allowSleep = false;
	box.position.set(0, 1, 0);
	world.addBody(elevator, box);

	for (let i = 0; i < 150; i++) world.step(1 / 60);
	elevator.velocity.set(1, 0, 0);
	for (let i = 0; i < 90; i++) world.step(1 / 60);

	assert.ok(box.position.x > 1.1 && box.position.x < 1.3);
	assert.ok(Math.abs(box.position.y - 0.5) <= 0.02);
});

test('fast moving crate wakes and displaces sleeping crate', () => {
	const world = makeWorld();
	const ground = platform(2);
	const target = crate();
	target.position.set(0, 1, 0);
	world.addBody(ground, target);

	for (let i = 0; i < 150; i++) world.step(1 / 60);

	const mover = crate();
	mover.position.set(-2, 0.5, 0);
	mover.velocity.set(6, 0, 0);
	world.addBody(mover);

	for (let i = 0; i < 40; i++) world.step(1 / 60);
	assert.notEqual(target.sleepState, Body.SLEEPING);
	assert.ok(Math.abs(target.position.x) > 0);
});
