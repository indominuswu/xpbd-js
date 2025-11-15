import * as THREE from "three";

export class PhysicsScene {
  constructor({ gravity = [0, -10, 0], dt = 1 / 60, numSubsteps = 5 } = {}) {
    this.config = Object.freeze({
      gravity,
      dt,
      numSubsteps,
    });

    this.state = {
      paused: true,
      timeSum: 0,
      frameSum: 0,
      timeFrames: 0,
    };

    this.objects = [];
    this.sphereColliders = [];

    this.view = {
      showEdges: true,
      showLabels: false,
      showArea: true,
    };
  }

  addObject(obj) {
    this.objects.push(obj);
  }

  addSphereCollider(sphere) {
    this.sphereColliders.push(sphere);
  }

  setPaused(v) {
    this.state.paused = v;
  }
  togglePaused() {
    this.state.paused = !this.state.paused;
  }

  showEdges(v) {
    this.view.showEdges = v;
    this.objects.forEach((o) => {
      if (o.edgeMesh && o.triMesh) {
        o.edgeMesh.visible = v;
        o.triMesh.visible = !v;
      }
    });
  }

  toggleEdges() {
    this.showEdges(!this.view.showEdges);
  }

  showLabels(v) {
    this.view.showLabels = v;
    this.objects.forEach((o) => {
      if (o.triLabels) o.triLabels.setVisible(v);
      if (o.edgeLabels) o.edgeLabels.setVisible(v);
      if (o.vertexLabels) o.vertexLabels.setVisible(v);
    });
  }

  showArea(v) {
    this.view.showArea = v;
    this.objects.forEach((o) => {
      if (o.areaLabels) o.areaLabels.setVisible(v);
    });
  }

  step({ onSimulationStep = (dt) => {}, onUpdateFrameTime = (ms) => {} }) {
    var startTime = performance.now();

    var sdt = this.config.dt / this.config.numSubsteps;
    for (var step = 0; step < this.config.numSubsteps; step++) {
      for (var i = 0; i < this.objects.length; i++) {
        this.objects[i].preSolve(sdt, this.config.gravity);
      }
      for (var i = 0; i < this.objects.length; i++) {
        const colliders = [];
        this.sphereColliders.forEach((sphere) => {
          colliders.push({
            sphereCenter: sphere.pos,
            sphereRadius: sphere.radius,
          });
        });
        this.objects[i].solve(sdt, colliders);
      }
      for (var i = 0; i < this.objects.length; i++) {
        this.objects[i].postSolve(sdt);
      }
    }
    for (var i = 0; i < this.objects.length; i++) {
      this.objects[i].endFrame();
    }

    var endTime = performance.now();
    this.state.timeSum += endTime - startTime;
    this.state.frameSum++;
    this.state.timeFrames++;

    onSimulationStep(this.config.dt);

    if (this.state.timeFrames > 10) {
      const avg = this.state.timeSum / this.state.timeFrames;
      onUpdateFrameTime(avg);
      this.state.timeFrames = 0;
      this.state.timeSum = 0;
    }
  }
}
