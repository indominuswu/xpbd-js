import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export class Grabber {
  constructor({
    canvas,
    threeScene,
    physics,
    rendererElement,
    runSimulationFunc,
  }) {
    this.raycaster = new THREE.Raycaster();
    this.raycaster.layers.set(1);
    this.raycaster.params.Line.threshold = 0.1;
    this.physicsObject = null;
    this.distance = 0.0;
    this.prevPos = new THREE.Vector3();
    this.vel = new THREE.Vector3();
    this.time = 0.0;
    this.threeScene = threeScene;
    this.physics = physics;
    this.runSimulationFunc = runSimulationFunc;
    this.mouseDown = false;
    this.onPointer = this.onPointer.bind(this);
    this.canvas = canvas;
    this.canvas.addEventListener("pointerdown", this.onPointer, false);
    this.canvas.addEventListener("pointermove", this.onPointer, false);
    this.canvas.addEventListener("pointerup", this.onPointer, false);

    this.controls = new OrbitControls(threeScene.camera, rendererElement);
    this.controls.zoomSpeed = 2.0;
    this.controls.panSpeed = 0.4;
    this.controls.target = new THREE.Vector3(0.0, 0.6, 0.0);
    this.controls.update();
  }
  onPointer(e) {
    e.preventDefault();
    if (e.type === "pointerdown") {
      this.start(e.clientX, e.clientY);
      this.mouseDown = true;
      if (this.physicsObject) {
        this.controls.saveState();
        this.controls.enabled = false;
      }
    } else if (e.type === "pointermove" && this.mouseDown) {
      this.move(e.clientX, e.clientY);
    } else if (e.type === "pointerup") {
      if (this.physicsObject) {
        this.end();
        this.controls.reset();
      }
      this.mouseDown = false;
      this.controls.enabled = true;
    }
  }
  increaseTime(dt) {
    this.time += dt;
  }
  updateRaycaster(x, y) {
    var rect = this.threeScene.renderer.domElement.getBoundingClientRect();
    this.mousePos = new THREE.Vector2();
    this.mousePos.x = ((x - rect.left) / rect.width) * 2 - 1;
    this.mousePos.y = -((y - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.mousePos, this.threeScene.camera);
  }
  start(x, y) {
    this.physicsObject = null;
    this.updateRaycaster(x, y);
    var intersects = this.raycaster.intersectObjects(
      this.threeScene.scene.children
    );
    if (intersects.length > 0) {
      var obj = intersects[0].object.userData;
      if (obj) {
        this.physicsObject = obj;
        this.distance = intersects[0].distance;
        var pos = this.raycaster.ray.origin.clone();
        pos.addScaledVector(this.raycaster.ray.direction, this.distance);
        this.physicsObject.startGrab(pos);
        this.prevPos.copy(pos);
        this.vel.set(0.0, 0.0, 0.0);
        this.time = 0.0;
        if (this.physics.state.paused) {
          this.runSimulationFunc();
        }
      }
    }
  }
  move(x, y) {
    if (this.physicsObject) {
      this.updateRaycaster(x, y);
      var pos = this.raycaster.ray.origin.clone();
      pos.addScaledVector(this.raycaster.ray.direction, this.distance);

      this.vel.copy(pos);
      this.vel.sub(this.prevPos);
      if (this.time > 0.0) this.vel.divideScalar(this.time);
      else this.vel.set(0.0, 0.0, 0.0);
      this.prevPos.copy(pos);
      this.time = 0.0;

      this.physicsObject.moveGrabbed(pos, this.vel);
    }
  }
  end(x, y) {
    if (this.physicsObject) {
      this.physicsObject.endGrab(this.prevPos, this.vel);
      this.physicsObject = null;
    }
  }
  dispose() {
    this.canvas.removeEventListener("pointerdown", this._onPointer);
    this.canvas.removeEventListener("pointermove", this._onPointer);
    this.canvas.removeEventListener("pointerup", this._onPointer);
  }
}
