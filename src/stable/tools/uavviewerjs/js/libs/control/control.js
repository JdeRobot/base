var GUI = GUI || {};

/*
 * GUI.Control's Constructor.
 * Params:
 *     - config:
 *       + id (canvas' id to show the control)
 */
GUI.Control = function (config){
    var conf = config || {};
   
   this.id = conf.id;
   this.WIDTH = 320;
   this.HEIGHT = 320;
   this.VIEW_ANGLE = 50;
   this.NEAR = 0.1;
   this.FAR = 5000;
   this.renderer =undefined;
   this.camera = undefined;
   this.controls = undefined;
   this.circle =undefined;
   this.position =new THREE.Vector3(0,0,0);
   
   this.ASPECT=this.WIDTH / this.HEIGHT;
   
   this.onPointerD = undefined;
   this.onPointerU = undefined;
   this.onPointerM = undefined;
   
   var self=this;
   
   
   
   /*
    * buildLine
    * It makes a line of color colorHex from src to dst
    */
   var  buildLine = function( src, dst, colorHex, dashed ) {
        var geom = new THREE.Geometry(),
            mat;

        if(dashed) {
                mat = new THREE.LineDashedMaterial({ linewidth: 2, color: colorHex, dashSize: 3, gapSize: 3 });
        } else {
                mat = new THREE.LineBasicMaterial({ linewidth: 2, color: colorHex });
        }

        geom.vertices.push( src.clone() );
        geom.vertices.push( dst.clone() );
        geom.computeLineDistances(); // This one is SUPER important, otherwise dashed lines will appear as simple plain lines

        var axis = new THREE.Line( geom, mat, THREE.LineSegments );

        return axis;

   };
   /*
    * buildJoyStick
    * It is a group of elements formed by two lines and a circle to handle speeds
    */
   var buildJoyStick= function(){
      var group = new THREE.Group();
      
      var geometry = new THREE.SphereGeometry( 5, 16, 16 );
      var material = new THREE.MeshBasicMaterial( {color: 0xff0000} );
      var point = new THREE.Mesh( geometry, material );
      group.add(point);
      var line1 = buildLine(new THREE.Vector3( 100, 1, 0 ), new THREE.Vector3( -100, 1, 0 ), 0x00FF00, false );
      group.add(line1);
      var line2 = buildLine(new THREE.Vector3( 0, 1, 100 ), new THREE.Vector3( 0, 1, -100 ), 0x00FF00, false );
      group.add(line2);
      
      return group;
   
   };
   
   
   var rendControl = function(){
         self.renderer.render(self.scene,self.camera);
      
      };
   
   this.onPointerUDefault = function (event){
         self.controls.onPointerUp(event);
      };
   
   this.onPointerDDefault = function (event){
         self.controls.onPointerDown(event);
      };
   
   this.onPointerMDefault = function (event){
         self.controls.onPointerMove(event);
         var pos = self.circle.position;
         self.position.copy(pos);
         
      };
   

   this.putListeners = function (){
      var onPU = self.onPointerU || self.onPointerUDefault;
      var onPM = self.onPointerM || self.onPointerMDefault;
      var onPD = self.onPointerD || self.onPointerDDefault;
      
      self.renderer.domElement.addEventListener( "mouseup", onPU, false );
      self.renderer.domElement.addEventListener( "touchend", onPU, false );
      self.renderer.domElement.addEventListener( "mousedown", onPD, false );
      self.renderer.domElement.addEventListener( "touchstart", onPD, false );
      self.renderer.domElement.addEventListener( "mousemove", onPM, false );
      self.renderer.domElement.addEventListener( "touchmove", onPM, false );
      
      self.controls.attach( self.circle );
   
   };
   this.removeListeners = function(){
      
      self.controls.detach( self.circle );
      var onPU = self.onPointerU || self.onPointerUDefault;
      var onPM = self.onPointerM || self.onPointerMDefault;
      var onPD = self.onPointerD || self.onPointerDDefault;
      
      self.renderer.domElement.removeEventListener( "mouseup", onPU);
      self.renderer.domElement.removeEventListener( "touchend", onPU);
      self.renderer.domElement.removeEventListener( "mousedown", onPD);
      self.renderer.domElement.removeEventListener( "touchstart", onPD);
      self.renderer.domElement.removeEventListener( "mousemove", onPM);
      self.renderer.domElement.removeEventListener( "touchmove", onPM);
   
   };

   this.initControl = function (){
      
      self.circle = buildJoyStick();
      self.controls = new THREE.JoyStickControls( self.camera, self.renderer.domElement );
      self.controls.addEventListener( 'change', rendControl );
   
      self.controls.children[0].visible=false;
         
      self.scene.add( self.circle );
      self.putListeners();
      self.scene.add( self.controls );
      
      var helper = new THREE.GridHelper( 50, 10 );
      helper.setColors( 0x0000ff, 0x808080 );
      self.scene.add(helper);
      
      self.renderer.render(self.scene,self.camera);
   };

   this.camera = new THREE.PerspectiveCamera(this.VIEW_ANGLE, this.ASPECT, this.NEAR, this.FAR);
   this.camera.position.set(0,100,0);
      
   this.camera.lookAt(new THREE.Vector3( 0,0,0 ));
   this.camera.rotation.z=Math.PI;

   this.renderer = new THREE.WebGLRenderer({canvas:document.getElementById(this.id)});
   this.renderer.setSize(this.WIDTH,this.HEIGHT);

   this.scene=new THREE.Scene();
};