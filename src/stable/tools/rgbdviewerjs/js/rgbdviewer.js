/*
 * RgbdViewer's Constructor.
 * Params:
 *     - config (contents client's Config)={serv,camid,fpsid,modelid}:
 *       + serv (server's direction and port)={dir:direction,port: port}
 *       + camid (canvas' id to show RGB camera)
 *       + camepname (name of camera endpoint, default cameraA)
 *       + fpscamid (id of element to show fps of RGB camera, is optional)
 *       + depthid (canvas' id to show depth camera)
 *       + depepname (name of camera endpoint, default cameraB)
 *       + fpsdepid (id of element to show fps of depth camera, is optional)
 *       + modelid (canvas' id to show 3D model)
 */
function RgbdViewer (config){
   //console.log(config);
   /*************************
    **** Public objects  ****
    *************************/
   this.serv=config.serv;
   this.camid=config.camid;
   this.fpscamid=config.fpscamid;
   this.depthid=config.depthid;
   this.fpsdepid=config.fpsdepid;
   this.modelid=config.modelid;
   
   this.camepname = config.camepname || "cameraA";
   this.depepname = config.depepname || "cameraB";
   
   
   /*************************
    **** Private objects ****
    *************************/
   var rgb=undefined;
   var depth = undefined;
   
   var cameraconf={
      position:new HPoint3D(0,0,500),
      yaw:0,
      pitch:0,
      roll:Math.PI,
      scale:new HPoint3D(0.1,0.1,0.1),
      focus: new HPoint3D(0,0,240)
   };
   var tphCamera = new TPinHoleCamera (cameraconf);
   
   var self=this;
   
   
   
   //Webgl variables
   //cloudPoints constants
   var SAMPLE=3;
   var MAX=640;
   
   //Canvas' dimensions
   var WIDTH = 640/SAMPLE;
   var HEIGHT = 480/SAMPLE;
   //camera's data
   var VIEW_ANGLE = 50; 
   var ASPECT = WIDTH / HEIGHT;
   var NEAR = 0.1;
   var FAR = 50000;
   //var WIDTH, HEIGHT, VIEW_ANGLE, ASPECT, NEAR, FAR;
   
   var rendergl, scenegl, cameragl;
   var plane_yx, plane_yzder, plane_yzizq, plane_xz;
   var controls; 
   var lastCP;
   var renderext;
   
   

   
   /*************************
    **** Private methods ****
    *************************/
   
   
   /*********** WebGL ******/
   
   /*
    * initGL
    * Prepare reconstruction with webgl
    */
   var initGL = function(){
      var canv = document.getElementById(self.modelid);
      cameragl=new THREE.PerspectiveCamera(VIEW_ANGLE, ASPECT, NEAR, FAR);

      rendergl=new THREE.WebGLRenderer({canvas:canv});

      scenegl=new THREE.Scene();

      //adding the camera on scene
      scenegl.add(cameragl);


      //controls
      controls = new THREE.TrackballControls( cameragl );

      controls.rotateSpeed = 2.0;
      controls.zoomSpeed = 2.2;
      controls.panSpeed = 1.8;

      controls.noZoom = false;
      controls.noPan = false;

      controls.staticMoving = true;
      controls.dynamicDampingFactor = 0.3;
      controls.enabled=false;

      controls.keys = [ 65, 83, 68 ];
      controls.target=new THREE.Vector3( 0,0,tphCamera.position.z );
                                        
      canv.addEventListener("mouseover", function(){
            controls.enabled=true;
      });
      
      canv.addEventListener("mouseout", function(){
            controls.enabled=false;
      });
   
      //everything is "drawn"  in the canvas
      rendergl.render(scenegl,cameragl);
      animation();
   }
   
   /*
    * drawCamera
    * Draw the cloud of points received from the server
    */
   var drawCamera = function (data,canvas,fps){
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

      //FPS
      if (data.fps){
         fps.html(Math.floor(data.fps));
      }
      
   };
   
   /*
    * drawCloudPoint
    * Draw the cloud of points received from the server
    */
   var drawCloudPoint = function (){
      if(!(depth.update && rgb.update)){return;}
      var depthd=depth.data;
      var rgbd=rgb.data;
      var points=[];
      var colors = [];
      
      
      var h,w;
      if (depthd.height>depthd.width){
         h=MAX;
         w=(h*depthd.width)/depthd.height;
      }else{
         w=MAX;
         h=(w*depthd.height)/depthd.width;
      }
      
      var color = new THREE.Color();
      var x=-depthd.width/2;
      var y=-depthd.height/2;
      var z=0;
      var a=0;
      
      //prepare the points and each point color
      for (var i=0; i<depthd.pixelData.length;i+=depthd.width*3*SAMPLE){
         x=-depthd.width/2;
         for (var j=0; j<depthd.width*3;j+=3*SAMPLE){
            z=((depthd.pixelData[i+j+1]<<8) + depthd.pixelData[i+j+2]);
            points[a]=x;
            points[a+1]=y;
            points[a+2]=z;

            color.setRGB(rgbd.pixelData[i+j]/255,rgbd.pixelData[i+j+1]/255,rgbd.pixelData[i+j+2]/255);

            colors[ a ]     =color.r;
            colors[ a+ 1 ] =color.g;
            colors[ a+ 2 ] = color.b;
            
            a+=3;
         
            x+=(w/depthd.width)*SAMPLE;
         }
         y+=(h/depthd.height)*SAMPLE;
      }
      
      var hpoints = conicProjectionCloudHPoint3D(cloudPoint2CloudHPoint3D(points),tphCamera);
      var cloudpoint = cloudHPoint3D2CloudPoint(hpoints);
   
   
      //I do the object using the points, its color and a material that shows the color of each point
      var geometry = new THREE.BufferGeometry();
      geometry.addAttribute( 'position', new THREE.BufferAttribute( new Float32Array(cloudpoint), 3 ) );
      geometry.addAttribute( 'color', new THREE.BufferAttribute( new Float32Array(colors), 3 ) );
   
      var material = new THREE.PointsMaterial( {  vertexColors: THREE.VertexColors} );
      image= new THREE.Points( geometry, material);
  
      if (lastCP){
         scenegl.remove(lastCP);
      }
      scenegl.add(image);
      lastCP=image;
      
      
      depth.update=false;
      rgb.update=false;

   }
   
   /*
    * animation
    * refresh the model
    */
   var animation = function (){
      requestAnimationFrame(animation);
      controls.update();
      render_modelo();
   }

   /*
    * render_modelo
    * Render the model
    */
   var render_modelo = function (){
      rendergl.render(scenegl,cameragl);
   }
   
   /*************************
    ** Privileged methods ***
    *************************/
   
   this.setConfig = function(conf){
      this.serv=conf.serv || this.serv;
      this.camid=conf.camid || this.camid;
      this.fpscamid=conf.fpscamid || this.fpscamid;
      this.depthid=conf.depthid || this.depthid;
      this.fpscdepid=conf.fpsdepid || this.fpscdepid;
      this.modelid=conf.modelid || this.modelid;

      this.camepname = conf.camepname || this.camepname;
      this.depepname = conf.depepname || this.depepname;
   
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
      if (!this.serv||!this.serv.dir||!this.serv.port) {
         alert("The server's data are not well defined");
         return;
      } 
      if (!this.camid) {
         alert("Not defined where the RGB image (camid)");
         return;
      } 
      if (!this.depthid) {
         alert("Not defined where the Depth image (depthid)");
         return;
      } 
      if (!this.modelid) {
         alert("Not defined where the Reconstruction (modelid)");
         return;
      } 
      
      initGL();
      
      rgb = new API.Camera({id:this.camid,
                           fpsid:this.fpscamid,
                           server:this.serv,
                           imgFormat:"RGB8",
                           epname:"cameraA"});
      rgb.update = false;
      rgb.canvas = document.getElementById(self.camid);
      rgb.fps = $('#'+this.fpscamid);
      rgb.onmessage= function (event){
            rgb.onmessageDefault(event);
            drawCamera(rgb.data,rgb.canvas,rgb.fps);
            rgb.update = true;
            drawCloudPoint();
      };
      
      depth = new API.Camera({id:this.depthid,
                           fpsid:this.fpscdepid,
                           server:this.serv,
                           imgFormat:"DEPTH8_16",
                           epname:"cameraB"});
      depth.update= false;
      depth.canvas = document.getElementById(self.depthid);
      depth.fps = $('#'+this.fpsdepid);
      depth.onmessage= function (event){
            depth.onmessageDefault(event);
            drawCamera(depth.data,depth.canvas,depth.fps);
            depth.update = true;
            drawCloudPoint();
      };
      
      
      rgb.connect();
      depth.connect();

      rgb.startStreaming();
      depth.startStreaming();
      
      
   };
   
   
   /*
    * stop
    * stop client 
    */
   this.stop= function () {
      rgb.deleteWork();
      depth.deleteWork();
      
   };
   /*
    * isrunning
    * Returns a boolean indicating if the client is running
    */
   this.isrunning= function () {
      return rgb.isRunning || depth.isRunning;
   };
   
   /*
    * restart
    * stops and start client
    */
   this.restart= function () {
      this.stop();
      this.start();
   };
}
