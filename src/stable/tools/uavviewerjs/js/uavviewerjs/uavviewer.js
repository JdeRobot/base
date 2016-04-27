/*
 * UavViewer's Constructor.
 * Params:
*     - config (contents client's Config)={serv,camid,fpsid,modelid}:
 *       + cam1serv (server's direction and port)={dir:direction,port: port}
 *       + cam1epname (name of camera left endpoint, default cam_sensor_left)
 *       + cam1id (canvas' id to show cam right)
 *       + cmdvelserv (server's direction and port)={dir:direction,port: port}
 *       + cmdvelepname (name of Motors endpoint, default Motors)
 *       + control1id (canvas' id to show controls)
 *       + control2id (canvas' id to show controls) 
 *       + modelid (canvas' id to show model of robot)
 *       + pose3dserv (server's direction and port)={dir:direction,port: port}
 *       + pose3depname (name of Motors endpoint, default Pose3DEncoders1)
 *       + takeoffbtnid (button to takeoff drone)
 *       + landbtnid (button to land drone)
 *       + togcambtnid (button to switch drone camera)
 *       + attitudeid, headingid, altimeterid, turn_coordinatorid (ids to show flying indicators)
 *       + extraserv (server's direction and port)={dir:direction,port: port}
 *       + extraepname (name of Motors endpoint, default Pose3D)
 */
function UavViewer (config){
   //console.log(config);
   /*************************
    **** Public objects  ****
    *************************/
   this.cam1serv=config.cam1serv;
   this.pose3dserv=config.pose3dserv;
   this.cmdvelserv=config.cmdvelserv;
   this.extraserv=config.extraserv;
   
   this.cam1id=config.cam1id;
   this.control1id=config.control1id;
   this.control2id=config.control2id;
   this.takeoffbtnid=config.takeoffbtnid;
   this.resetbtnid=config.resetbtnid;
   this.landbtnid=config.landbtnid;
   this.togcambtnid=config.togcambtnid;
   this.stopbtnid=config.stopbtnid;
   this.modelid=config.modelid;
   this.attitudeid = config.attitudeid;
   this.headingid = config.headingid;
   this.altimeterid = config.altimeterid;
   this.turn_coordinatorid = config.turn_coordinatorid;
   
   this.cmdvelepname=config.cmdvelepname || "CMDVel";
   this.cam1epname=config.cam1epname || "Camera";
   this.extraepname = config.extraepname || "Extra";
   this.pose3depname = config.pose3depname || "Pose3D";

   var model = {id: this.modelid,
                WIDTH: 320,
                HEIGHT: 320,
                VIEW_ANGLE: 50,
                NEAR: 0.1,
                FAR: 5000,
                robot:undefined,
                renderer:undefined,
                camera: undefined,
                controls: undefined,
   };
   model.ASPECT=model.WIDTH / model.HEIGHT;
   
   var extra;
   var camera1;
   var pose3d;
   var cmdvel;
   var timeout = 10000;
   var control1;
   var control2;
   
   var self=this;
   
   var cmdSend = {
         linearX:0,
		 linearY:0,
		 linearZ:0,
		 angularX:0,
		 angularY:0,
		 angularZ:0		
   };
   
   var toDegrees = 180/Math.PI;
   
   var attitude;
   var heading;
   var altimeter;
   var turn_coordinator;
      

   
   /*************************
    **** Private methods ****
    *************************/
    var calcDist= function (num1, num2){
         var max = Math.max(num1,num2);
         var min = Math.min(num1,num2);
         
         return max-min;
      };
   
   var drawCamera = function (data,canvas){
      //camera
      var canvas2 = document.createElement('canvas');
      var ctx2=canvas2.getContext("2d");       
      var imgData=ctx2.getImageData(0,0,data.width,data.height);
      ctx2.canvas.width=data.width;
      ctx2.canvas.height=data.height;
      var ctx=canvas.getContext("2d");
      imgData.data.set(data.imgData);
      ctx2.putImageData(imgData,0,0);
      ctx.drawImage(canvas2, 0, 0,ctx.canvas.width,ctx.canvas.height);
   };
   

   var initControls = function (){
      var v = 0.3,
          w = 0.3;
      var distSend = v/90;
      
      function sendlxly (x,y) {
         var lx = v*y;
         var ly = v*x*(-1);
         var send = false;
         
         if (calcDist(ly,cmdSend.linearY)>=distSend){
            cmdSend.linearY = ly;
            send = true;
         }
         if (calcDist(lx,cmdSend.linearX)>=distSend){
            cmdSend.linearX = lx;
            send = true;
         }
         if (send){
            cmdvel.setCmdVel(cmdSend);
         }
         //console.log(cmdSend);
      }
      
      function sendlzaz (x,y) {
         var lz = v*y;
         var az = w*x*(-1);
         
         var send = false;
         
         if (calcDist(az,cmdSend.angularZ)>=distSend){
            cmdSend.angularZ = az;
            send = true;
         }
         
         if (calcDist(lz,cmdSend.linearZ)>=distSend){
            cmdSend.linearZ = lz;
            send = true;
         }
         
         if (send){
            cmdvel.setCmdVel(cmdSend);
         }
      }
      
      control1 = new GUI.Joystick({id:self.control1id, onMove: sendlxly, onUp: sendlxly});
      control2 = new GUI.Joystick({id:self.control2id, onMove: sendlzaz, onUp: sendlzaz});
      
   };
   
   
    var initModel = function (){
      var canv = document.getElementById(model.id);
      model.camera = new THREE.PerspectiveCamera(model.VIEW_ANGLE, model.ASPECT, model.NEAR, model.FAR);
      model.camera.position.set( -5, 5, 2 );
      
      model.camera.lookAt(new THREE.Vector3( 0,0,0 ));
      
      model.renderer = new THREE.WebGLRenderer({canvas:canv});
      model.renderer.setClearColor( 0xffffff);
      
      model.scene=new THREE.Scene();
       
      //controls
      model.controls = new THREE.TrackballControls( model.camera );

      model.controls.rotateSpeed = 2.0;
      model.controls.zoomSpeed = 2.2;
      model.controls.panSpeed = 1.8;

      model.controls.noZoom = false;
      model.controls.noPan = false;

      model.controls.staticMoving = true;
      model.controls.dynamicDampingFactor = 0.3;
      model.controls.enabled=false;

      model.controls.keys = [ 65, 83, 68 ];
       
      canv.addEventListener("mouseover", function(){
            model.controls.enabled=true;
      });
      
      canv.addEventListener("mouseout", function(){
            model.controls.enabled=false;
      });
       
      //lights
      var light = new THREE.HemisphereLight( 0xffffbb, 0x080820, 1 );
      light.position.set( 0, 500, 0 );
      model.scene.add(light);
       
       
      // Grid
      var size = 25;
      var step = 1;

      var gridHelper = new THREE.GridHelper( size, step );	
      model.scene.add( gridHelper );
   
      var axisHelper = new THREE.AxisHelper( 2 );
      model.scene.add( axisHelper );
      
      //ground
      var groundMat = new THREE.MeshBasicMaterial({color:0xd8d8d8});
      var groundGeo = new THREE.PlaneGeometry(50,50);
      var ground = new THREE.Mesh(groundGeo,groundMat); 
      ground.position.y = -0.05; //lower it 
      ground.rotation.x = -Math.PI/2; //-90 degrees around the xaxis  
      
      model.scene.add(ground); 
                                        
      var modelAnimation = function(){
         requestAnimationFrame(modelAnimation);
         model.controls.update();
         model.renderer.render(model.scene,model.camera);
         
      };
      
      var loader = new API.RobotLoader();
       
      loader.loadQuadrotor(0.05,function () {
               model.robot=loader.robot;
               model.scene.add( model.robot );
         
               pose3d = new API.Pose3D({server:self.pose3dserv,epname:self.pose3depname});
               pose3d.onmessage= function (event){
                  pose3d.onmessageDefault(event);
                  model.robot.position.set(pose3d.data.x,pose3d.data.z,-pose3d.data.y);
                  model.robot.rotation.set(pose3d.data.pitch,pose3d.data.yaw,pose3d.data.roll);
                  model.robot.updateMatrix();
                  model.renderer.render(model.scene,model.camera);
                  // Attitude update
                  attitude.setRoll(-pose3d.data.roll * toDegrees);
                  attitude.setPitch(-pose3d.data.pitch * toDegrees);

                   // Altimeter update
                   altimeter.setAltitude(pose3d.data.z*100);

                   // TC update
                   turn_coordinator.setTurn(-pose3d.data.roll * toDegrees);

                   // Heading update
                   heading.setHeading(pose3d.data.yaw * toDegrees);
               };
         
               pose3d.timeoutE=timeout;
         
               pose3d.connect();
               pose3d.startStreaming();
               
               modelAnimation();
	        });
   };
   
   /*
    * initGL
    * Prepare reconstruction with webgl
    */
   var initGL = function(){
      initControls();
      if (self.modelid){
         initModel();
      }
   }

   /*************************
    ** Privileged methods ***
    *************************/
   
   this.setConfig = function(conf){
      this.cam1serv=conf.cam1serv || this.cam1serv;
   
      this.cam1id=conf.cam1id || this.cam1id;
      this.controlid=conf.controlid || this.controlid;
   
      this.cam1epname=conf.cam1epname || this.cam1epname;
   
   };
   
   /*
    * start
    * run client 
    */
   this.start= function(){
      //worker, serv, camid checks
      if (!window.Worker) {
         alert("This application does not work in this browser");
         return;
      }
      if (!this.cam1serv||!this.cam1serv.dir||!this.cam1serv.port) {
         alert("The cam1serv's data are not well defined");
         return;
      } 
      if (!this.pose3dserv||!this.pose3dserv.dir||!this.pose3dserv.port) {
         alert("The pose3dserv's data are not well defined");
         return;
      } 
      if (!this.cam1id) {
         alert("Not defined cam1id");
         return;
      } 
      if (!this.control1id) {
         alert("Not defined controlid");
         return;
      }
      if (!this.control2id) {
         alert("Not defined controlid");
         return;
      }
      if (!this.modelid) {
         alert("Not defined modelid");
         return;
      }
      var sizeInd = $("#camView").height()/5;
      attitude = $.flightIndicator('#'+this.attitudeid, 'attitude', {showBox : false, size: sizeInd}); // Horizon
      heading = $.flightIndicator('#'+this.headingid, 'heading', {showBox:false, size: sizeInd}); // Compass
      altimeter = $.flightIndicator('#'+this.altimeterid, 'altimeter', {showBox : false, size: sizeInd}); // drone altitude
      turn_coordinator = $.flightIndicator('#'+this.turn_coordinatorid, 'turn_coordinator', {showBox : false, size: sizeInd});
      
      //pose3d in initModel
      cmdvel = new API.CmdVel ({server:this.cmdvelserv,epname:this.cmdvelepname});
      cmdvel.connect();
      extra = new API.ArDroneExtra ({server:this.extraserv,epname:this.extraepname});
      $('#'+this.takeoffbtnid).on('click', function(){
         extra.takeoff();
	  });
      $('#'+this.togcambtnid).on('click', function(){
         extra.toggleCam();
	  });
       $('#'+this.landbtnid).on('click', function(){
         extra.land();
	  });
      extra.connect();
      camera1 = new API.Camera ({server:this.cam1serv,epname:this.cam1epname});
      camera1.canvas = document.getElementById(self.cam1id);
      camera1.onmessage= function (event){
            camera1.onmessageDefault(event);
            drawCamera(camera1.data,camera1.canvas);
      };
      camera1.timeoutE=timeout;
      camera1.connect();
      camera1.startStreaming();
      initGL();
   };
   
   this.stop = function(){
      pose3d.disconnect();
      camera1.disconnect();
      cmdvel.disconnect();
      extra.disconnect();
   
   };
   
   /*
    * isrunning
    * Returns a boolean indicating if the client is running
    */
   this.isRunning= function () {
      return camera1.isRunning || pose3d.isRunning || cmdvel.isRunning || extra.isRunning;
   };
   
   this.resizeCameraModel= function(){
      model.camera.aspect = model.renderer.domElement.width / model.renderer.domElement.height;
      model.camera.updateProjectionMatrix();
      model.renderer.render(model.scene,model.camera);
   
   };
}
