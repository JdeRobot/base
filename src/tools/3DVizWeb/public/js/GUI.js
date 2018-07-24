
var camera, scene, renderer, controls;
var  axes, grid, particles;
var rotationx = 0.0;
var rotationy = 0.0;
var toDegrees = 180/Math.PI;


			function init() {
				camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 1000 );
				camera.position.z = config.camera.z;
        camera.position.y = config.camera.y;
        camera.position.x = config.camera.x;
				scene = new THREE.Scene();
				renderer = new THREE.WebGLRenderer();
				renderer.setSize( window.innerWidth, window.innerHeight);
        renderer.setClearColor(0x58D3F7);
				document.getElementById("canvas").appendChild( renderer.domElement );
				controls = new THREE.OrbitControls(camera, renderer.domElement);
				window.addEventListener( 'resize', onWindowResize, false );
				var ambientLight = new THREE.AmbientLight( 0xffffff, 0.4 );
				scene.add( ambientLight );
				var light = new THREE.PointLight( 0xffffff, 1, 100 );
				light.position.set( 10, 10, 10 );
				scene.add( light );
				var light = new THREE.PointLight( 0xffffff, 1, 100 );
				light.position.set( 20, 20, 20 );
				scene.add( light );
				var light = new THREE.PointLight( 0xffffff, 1, 100 );
				light.position.set( 30, 30, 30 );
				scene.add( light );
				var light = new THREE.PointLight( 0xffffff, 1, 100 );
				light.position.set( 40, 40, 40 );
				scene.add( light );
			}
			function onWindowResize() {
				camera.aspect = window.innerWidth / window.innerHeight;
				camera.updateProjectionMatrix();
				renderer.setSize( window.innerWidth, window.innerHeight );
			}
			function animate() {
				requestAnimationFrame( animate );
				renderer.render( scene, camera );
			}

			function addPoint (point){
				var geometry = new THREE.Geometry();
				geometry.vertices.push( new THREE.Vector3(point.x,point.z,point.y));
				var sprite = new THREE.TextureLoader().load("img/disc.png");
				var material = new THREE.PointsMaterial( { size: config.pointsize, sizeAttenuation: false, map: sprite, alphaTest: 0.5, transparent: true } );
				material.color.setRGB( point.r, point.g, point.b);
				var particles = new THREE.Points( geometry, material );
				particles.name ="points";
				scene.add( particles );
			}

			function addLine(segment,name){
				var geometry = new THREE.Geometry();
				geometry.vertices.push(
						new THREE.Vector3(segment.seg.fromPoint.x,segment.seg.fromPoint.z,segment.seg.fromPoint.y),
						new THREE.Vector3(segment.seg.toPoint.x,segment.seg.toPoint.z,segment.seg.toPoint.y),
						new THREE.Vector3(segment.seg.fromPoint.x,segment.seg.fromPoint.z,segment.seg.fromPoint.y));
				var material = new THREE.LineBasicMaterial();
				material.color.setRGB(segment.c.r,segment.c.g, segment.c.b);
				if (name != "plane"){
					material.linewidth = config.linewidth;
				}
				line = new THREE.Line(geometry,material);
				line.name = name;
				scene.add(line);
			}

			function addGrid(){
				grid = new THREE.GridHelper( 1000, 100, 0x888888, 0x888888);
				grid.position.set(0,-0.1,0);
				scene.add(grid);
			}

			function deleteObj(name){
				var selectedObject = scene.getObjectByName(name);
				while (selectedObject != null) {
					scene.remove(selectedObject);
					selectedObject = scene.getObjectByName(name);
				}
			}

			function addObj(obj){
				var type = obj.obj.split(":");
				if (type[0] == "https" ){
					var url = obj.obj
				} else{
					var file = new Blob([obj.obj], {type:'text/plain'});
					var url  = window.URL.createObjectURL(file);
				}
				if (obj.format == "obj"){
					loadObj(url, obj)
				} else if (obj.format == "dae") {
					loadDae(url,obj);
				}
			}

			function loadDae (url,obj){
				var loader = new THREE.ColladaLoader();
				loader.load(url, function (collada) {
					var avatar = collada.scene;
					avatar.name = obj.id;
					scene.add( avatar );
				} );
			}

			function loadObj(url,obj){
				var loader = new THREE.OBJLoader();
				loader.load(
					url,
					function(object){
						object.name = obj.id;
						scene.add(object);
					},
					function (xhr){
					},
					function (error){
						console.log(error);
					}
				);
			}

			function moveObj(pose3d){
				selectedObject = scene.getObjectByName(pose3d.id);
				selectedObject.position.set(pose3d.x,pose3d.y,pose3d.z);
				selectedObject.rotation.set(pose3d.rx*toDegrees,pose3d.ry * toDegrees, pose3d.rz * toDegrees);
			}

      function webGLStart (){
        init();
				//addGrid();
  			animate();
				startWorker();
      }
