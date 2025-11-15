import * as THREE from "three";

export class ThreeScene {
  constructor(canvas) {
    this.canvas = canvas;

    // --- Scene ----------------------------------------------------
    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    // this.scene.background = new THREE.Color(0xcccccc);
    this.scene.background = new THREE.Color(0x222222);

    const helper = new THREE.GridHelper(20, 20);
    helper.material.opacity = 1.0;
    helper.material.transparent = true;
    helper.position.set(0, 0.002, 0);
    this.scene.add(helper);

    // --- Lights ------------------------------------------------------
    let dirLight = new THREE.DirectionalLight(0xffffff, 3);
    dirLight.position.set(1, 3, 1);
    dirLight.castShadow = true;

    // 影の解像度など調整（必要なら）
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.1;
    dirLight.shadow.camera.far = 50;

    this.scene.add(dirLight);

    // let lightHelper = new THREE.DirectionalLightHelper(dirLight, 0.2);
    // this.scene.add(lightHelper);

    dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(-2, 3, -1);
    dirLight.castShadow = true;

    // 影の解像度など調整（必要なら）
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.1;
    dirLight.shadow.camera.far = 50;

    this.scene.add(dirLight);

    // lightHelper = new THREE.DirectionalLightHelper(dirLight, 0.2);
    // this.scene.add(lightHelper);

    // --- Renderer -------------------------------------------------
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.shadowMap.enabled = true;
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.canvas.appendChild(this.renderer.domElement);

    // --- Camera ---------------------------------------------------
    this.camera = new THREE.PerspectiveCamera(
      70,
      window.innerWidth / window.innerHeight,
      0.01,
      100
    );
    this.camera.position.set(0, 1, 1);
    this.camera.updateMatrixWorld();
    this.scene.add(this.camera);

    // --- Resize handler -------------------------------------------
    this._onResize = this._onResize.bind(this);
    window.addEventListener("resize", this._onResize, false);
  }

  renderFrame() {
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    window.removeEventListener("resize", this._onResize);
    this.renderer.dispose();
  }

  _onResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }
}
