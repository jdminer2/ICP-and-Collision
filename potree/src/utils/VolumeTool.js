
import * as THREE from "../../libs/three.js/build/three.module.js";
import {Volume, BoxVolume} from "./Volume.js";
import {Utils} from "../utils.js";
import { EventDispatcher } from "../EventDispatcher.js";

export class VolumeTool extends EventDispatcher{
	constructor (viewer) {
		super();

		this.viewer = viewer;
		this.renderer = viewer.renderer;

		this.addEventListener('start_inserting_volume', e => {
			this.viewer.dispatchEvent({
				type: 'cancel_insertions'
			});
		});

		this.scene = new THREE.Scene();
		this.scene.name = 'scene_volume';

		this.viewer.inputHandler.registerInteractiveScene(this.scene);

		this.onRemove = e => {
			this.scene.remove(e.volume);
		};

		this.onAdd = e => {
			this.scene.add(e.volume);
		};

		for(let volume of viewer.scene.volumes){
			this.onAdd({volume: volume});
		}

		this.viewer.inputHandler.addEventListener('delete', e => {
			let volumes = e.selection.filter(e => (e instanceof Volume));
			volumes.forEach(e => this.viewer.scene.removeVolume(e));
		});

		viewer.addEventListener("render.pass.scene", e => this.render(e));
		viewer.addEventListener("scene_changed", this.onSceneChange.bind(this));

		viewer.scene.addEventListener('volume_added', this.onAdd);
		viewer.scene.addEventListener('volume_removed', this.onRemove);
	}

	onSceneChange(e){
		if(e.oldScene){
			e.oldScene.removeEventListeners('volume_added', this.onAdd);
			e.oldScene.removeEventListeners('volume_removed', this.onRemove);
		}

		e.scene.addEventListener('volume_added', this.onAdd);
		e.scene.addEventListener('volume_removed', this.onRemove);
	}

	startInsertion (args = {}) {
		let volume;
		if(args.type){
			volume = new args.type();
		}else{
			volume = new BoxVolume();
		}
		
		volume.clip = args.clip || false;
		volume.name = args.name || 'Volume';

		this.dispatchEvent({
			type: 'start_inserting_volume',
			volume: volume
		});

		this.viewer.scene.addVolume(volume);
		this.scene.add(volume);

		let cancel = {
			callback: null
		};

		let drag = e => {
			let camera = this.viewer.scene.getActiveCamera();
			
			let I = Utils.getMousePointCloudIntersection(
				e.drag.end, 
				this.viewer.scene.getActiveCamera(), 
				this.viewer, 
				this.viewer.scene.pointclouds, 
				{pickClipped: false});

			if (I) {
				volume.position.copy(I.location);

				let wp = volume.getWorldPosition(new THREE.Vector3()).applyMatrix4(camera.matrixWorldInverse);
				// let pp = new THREE.Vector4(wp.x, wp.y, wp.z).applyMatrix4(camera.projectionMatrix);
				let w = Math.abs((wp.z / 5));
				volume.scale.set(w, w, w);
			}
		};

		let drop = e => {
			volume.removeEventListener('drag', drag);
			volume.removeEventListener('drop', drop);

			cancel.callback();
		};

		cancel.callback = e => {
			volume.removeEventListener('drag', drag);
			volume.removeEventListener('drop', drop);
			this.viewer.removeEventListener('cancel_insertions', cancel.callback);
		};

		volume.addEventListener('drag', drag);
		volume.addEventListener('drop', drop);
		this.viewer.addEventListener('cancel_insertions', cancel.callback);

		this.viewer.inputHandler.startDragging(volume);

		return volume;
	}

	render(params){
		const renderer = this.viewer.renderer;

		const oldTarget = renderer.getRenderTarget();
		
		if(params.renderTarget){
			renderer.setRenderTarget(params.renderTarget);
		}
		renderer.render(this.scene, this.viewer.scene.getActiveCamera());
		renderer.setRenderTarget(oldTarget);
	}

}
