const assert = require("node:assert/strict");
const { driveAxes } = require("./controls.js");
assert.deepEqual(driveAxes(new Set(["KeyW", "KeyA"])), { forward: 1, turn: 1 });
assert.deepEqual(driveAxes(new Set(["KeyW", "ArrowUp"])), { forward: 1, turn: 0 });
assert.deepEqual(driveAxes(new Set(["KeyW", "KeyS", "KeyA", "KeyD"])), { forward: 0, turn: 0 });
assert.deepEqual(driveAxes(new Set(["ArrowDown", "ArrowRight"])), { forward: -1, turn: -1 });
assert.deepEqual(driveAxes(new Set()), { forward: 0, turn: 0 });
console.log("Robot control axis tests passed");
