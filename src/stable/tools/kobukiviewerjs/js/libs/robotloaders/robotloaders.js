var API = API || {};


API.RobotLoader = function (){
   
   this.manager = new THREE.LoadingManager();
   
   this.manager.onProgress = function ( item, loaded, total ) {
	     console.log( item, loaded, total );
   };
   this.manager.onError = function ( item, loaded, total ) {
	     alert("Error loading Pioneer");
   };
   
   this.robot = undefined;
   this.minYPos =undefined;
   
   var self = this;
   
   
   this.loadPioneer = function (scale,onLoad){
      this.minYPos = 0.11;
      this.robot = new THREE.Group();
      var loaded = 2;
      this.manager.onLoad = onLoad;
      var chassisLoader = new THREE.ColladaLoader(self.manager);
      chassisLoader.options.convertUpAxis = true;
      chassisLoader.load(
	        'js/libs/robotloaders/pioneer/chassis.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.y=(self.minYPos+0.05)*scale;
               obj.updateMatrix();
       
               self.robot.add(obj);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var wheelLoader = new THREE.ColladaLoader(self.manager);
      wheelLoader.options.convertUpAxis = true;
      wheelLoader.load(
	        'js/libs/robotloaders/pioneer/wheel.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               var obj2 = obj.clone();
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.z=-0.17*scale;
               obj.position.x=0.1*scale;
               obj.position.y=self.minYPos*scale;
               obj.updateMatrix();
               
               obj2.scale.x =obj2.scale.y = obj2.scale.z = scale;
               obj2.position.z=0.17*scale;
               obj2.position.x=0.1*scale;
               obj2.position.y=self.minYPos*scale;
               obj2.rotation.y=Math.PI;
               obj2.updateMatrix();
       
               self.robot.add(obj);
               self.robot.add(obj2);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
   
   
   };
   
   this.loadKobuki = function (scale,onLoad){
      this.minYPos = 0;
      this.robot = new THREE.Group();
      var loaded = 6;
      this.manager.onLoad = onLoad;
      
      //wheels
      //front
      var geometry1 = new THREE.SphereGeometry( 0.009*scale, 32, 32 );
      var material1 = new THREE.MeshBasicMaterial( {color: 0x000000} );
      var sphere1 = new THREE.Mesh( geometry1, material1 );
      sphere1.position.y=(self.minYPos+0.017000)*scale;
      sphere1.position.x=(0.130000)*scale;
      self.robot.add( sphere1 );
      
      //rear
      var geometry2 = new THREE.SphereGeometry( 0.0075*scale, 32, 32 );
      var material2 = new THREE.MeshBasicMaterial( {color: 0x000000} );
      var sphere2 = new THREE.Mesh( geometry2, material2 );
      sphere2.position.y=(self.minYPos+0.017000)*scale;
      sphere2.position.x=(-0.130000)*scale;
      self.robot.add( sphere2 );
      
      //left
      var geometry3 = new THREE.SphereGeometry( 0.033*scale, 32, 32 );
      var material3 = new THREE.MeshBasicMaterial( {color: 0x000000} );
      var sphere3 = new THREE.Mesh( geometry3, material3 );
      sphere3.position.y=(self.minYPos+0.032000)*scale;
      sphere3.position.z=(0.130000)*scale;
      self.robot.add( sphere3 );
      
      var geometry4 = new THREE.SphereGeometry( 0.033*scale, 32, 32 );
      var material4= new THREE.MeshBasicMaterial( {color: 0x000000} );
      var sphere4 = new THREE.Mesh( geometry4, material4 );
      sphere4.position.y=(self.minYPos+0.032000)*scale;
      sphere4.position.z=(-0.130000)*scale;
      self.robot.add( sphere4 );
      
      var bodyLoader = new THREE.ColladaLoader(self.manager);
      bodyLoader.options.convertUpAxis = true;
      bodyLoader.load(
	        'js/libs/robotloaders/kobuki/create_body.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
            
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.y=(self.minYPos+0.047800)*scale;
               obj.updateMatrix();
       
               self.robot.add(obj);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      
      var plate0Loader = new THREE.ColladaLoader(self.manager);
      plate0Loader.options.convertUpAxis = true;
      plate0Loader.load(
	        'js/libs/robotloaders/kobuki/plate_0_logo.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.y=(self.minYPos+0.084757)*scale;
               obj.position.x=(-0.043340)*scale;
               obj.updateMatrix();
       
               self.robot.add(obj);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var plate1Loader = new THREE.ColladaLoader(self.manager);
      plate1Loader.options.convertUpAxis = true;
      plate1Loader.load(
	        'js/libs/robotloaders/kobuki/plate_1_logo.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               var obj2 = obj.clone();
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.y=(self.minYPos+0.141907)*scale;
               obj.position.x=(-0.002660)*scale;
               obj.updateMatrix();
               
               obj2.scale.x =obj2.scale.y = obj2.scale.z = scale;
               obj2.position.y=(self.minYPos+0.199108)*scale;
               obj2.position.x=(-0.002660)*scale;
               obj2.updateMatrix();
       
               self.robot.add(obj);
               self.robot.add(obj2);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var plate2Loader = new THREE.ColladaLoader(self.manager);
      plate2Loader.options.convertUpAxis = true;
      plate2Loader.load(
	        'js/libs/robotloaders/kobuki/plate_2_logo.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.position.y=(self.minYPos+0.405457)*scale;
               obj.position.x=(-0.015820)*scale;
               obj.updateMatrix();
       
               self.robot.add(obj);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var spacerLoader = new THREE.ColladaLoader(self.manager);
      spacerLoader.options.convertUpAxis = true;
      spacerLoader.load(
	        'js/libs/robotloaders/kobuki/68-02403-125_Spacer.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               obj.scale.x =obj.scale.y = obj.scale.z = 0.001*scale;
               obj.position.y=(self.minYPos+0.079992)*scale;
               obj.rotation.x=-Math.PI/2; //if I don't do it the object is shown in vertical
               
               var obj2 = obj.clone();
               var obj3 = obj.clone();
               var obj4 = obj.clone();

               obj.position.x=(-0.002540)*scale;
               obj.position.z=(0.111468)*scale;
               obj.updateMatrix();
               
               obj2.position.x=(-0.002540)*scale;
               obj2.position.z=(-0.111468)*scale;
               obj2.updateMatrix();
               
               obj3.position.x=(-0.072390)*scale;
               obj3.position.z=(-0.111468)*scale;
               obj3.updateMatrix();
               
               obj4.position.x=(-0.072390)*scale;
               obj4.position.z=(0.111468)*scale;
               obj4.updateMatrix();
       
               self.robot.add(obj);
               self.robot.add(obj2);
               self.robot.add(obj3);
               self.robot.add(obj4);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var standoffLoader = new THREE.ColladaLoader(self.manager);
      standoffLoader.options.convertUpAxis = true;
      standoffLoader.load(
	        'js/libs/robotloaders/kobuki/68-04552-2000-RA_Turtlebot_M-F_Standoff.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               obj.scale.x =obj.scale.y = obj.scale.z = 0.001*scale;
               obj.position.y=(self.minYPos+0.087980)*scale;
               obj.rotation.x=-Math.PI/2; //if I don't do it the object is shown in vertical
               
               var obj1 = obj.clone();
               var obj2 = obj.clone();
               var obj3 = obj.clone();
               
               var obj4 = obj.clone();
               obj4.position.y=(self.minYPos+0.145130)*scale;
               var obj5 = obj4.clone();
               var obj6 = obj4.clone();
               var obj7 = obj4.clone();
               
               obj.position.x=(0.067640)*scale;
               obj.position.z=(0.131420)*scale;
               
               obj.updateMatrix();
               
               obj1.position.x=(0.067640)*scale;
               obj1.position.z=(-0.131420)*scale;
               obj1.updateMatrix();
               
               obj2.position.x=(-0.052832)*scale;
               obj2.position.z=(-0.131420)*scale;
               obj2.updateMatrix();
               
               obj3.position.x=(-0.052832)*scale;
               obj3.position.z=(0.131420)*scale;
               obj3.updateMatrix();
               

               obj4.position.x=(0.067640)*scale;
               obj4.position.z=(0.131420)*scale;
               obj4.updateMatrix();
               
               obj5.position.x=(-0.052832)*scale;
               obj5.position.z=(-0.131420)*scale;
               obj5.updateMatrix();
               
               obj6.position.x=(0.067640)*scale;
               obj6.position.z=(-0.131420)*scale;
               obj6.updateMatrix();
               
               obj7.position.x=(-0.052832)*scale;
               obj7.position.z=(0.131420)*scale;
               obj7.updateMatrix();
               
       
               self.robot.add(obj);
               self.robot.add(obj1);
               self.robot.add(obj2);
               self.robot.add(obj3);
               self.robot.add(obj4);
               self.robot.add(obj5);
               self.robot.add(obj6);
               self.robot.add(obj7);
               
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
      var standoff8Loader = new THREE.ColladaLoader(self.manager);
      standoff8Loader.options.convertUpAxis = true;
      standoff8Loader.load(
	        'js/libs/robotloaders/kobuki/68-02421-8000-RA_Turtlebot_F-F_Standoff.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               obj.scale.x =obj.scale.y = obj.scale.z = 0.001*scale;
               obj.position.y=(self.minYPos+0.202280)*scale;
               obj.rotation.x=-Math.PI/2; //if I don't do it the object is shown in vertical
               
               var obj2 = obj.clone();
               var obj3 = obj.clone();
               var obj4 = obj.clone();
               
               obj.position.x=(0.067640)*scale;
               obj.position.z=(0.131420)*scale;
               obj.updateMatrix();
               
               obj2.position.x=(0.067640)*scale;
               obj2.position.z=(-0.131420)*scale;
               obj2.updateMatrix();
               
               obj3.position.x=(-0.052832)*scale;
               obj3.position.z=(-0.131420)*scale;
               obj3.updateMatrix();
               
               obj4.position.x=(-0.052832)*scale;
               obj4.position.z=(0.131420)*scale;
               obj4.updateMatrix();
       
               self.robot.add(obj);
               self.robot.add(obj2);
               self.robot.add(obj3);
               self.robot.add(obj4);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
   
   
   };
   
   this.loadQuadrotor = function (scale,onLoad){
      this.minYPos = 0;
      this.robot = new THREE.Group();
      var loaded = 1;
      this.manager.onLoad = onLoad;
      var chassisLoader = new THREE.ColladaLoader(self.manager);
      chassisLoader.options.convertUpAxis = true;
      chassisLoader.load(
	        'js/libs/robotloaders/quadrotor/red_quadrotor.dae',
	        function ( collada ) {
       
               var obj = collada.scene;
               
               obj.scale.x =obj.scale.y = obj.scale.z = scale;
               obj.updateMatrix();
       
               self.robot.add(obj);
               loaded--;
               if (loaded==0){
                  onLoad();
               }
	        });
      
   
   };
   


};