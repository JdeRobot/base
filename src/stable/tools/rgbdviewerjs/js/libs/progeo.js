/*
    * HPoint2D
    * Normalized 2D point class
    * Params:
    *     - x,y,h (point values)
    */
function HPoint2D (x,y,h){
   this.x = x || 0;
   this.y = y || 0;
   this.h = h || 1;
   
   this.toWebgl= function (){
      return [this.x,this.y]; // Returns the coordinates in the form of array to be able to be represented with webgl

   };
}

/*
    * HPoint3D
    * Normalized 3D point class
    * Params:
    *     - x,y,z,h (point values)
    */
function HPoint3D(x,y,z,h){
   this.x = x || 0;
   this.y = y || 0;
   this.z = z || 0;
   this.h = h || 1;
   
   this.toWebgl= function (){
      return [this.x,this.y,this.z]; // Returns the coordinates in the form of array to be able to be represented with webgl
   };
}

/*
    * TPinHoleCamera
    * class that representing the sensor (camera)
    * Params:
    *   - config:
    *     + position (HPoint3D)
    *     + yaw, pitch, roll (orientation in rads)
    *     + scale (HPoint3D): scale for transform
    *     + focus (HPoint3D): focus of sensor
    */
function TPinHoleCamera(config){
   var conf = config || {};
   this.position = conf.position || new HPoint3D();
   this.yaw = conf.yaw || 0; //in rads
   this.pitch = conf.pitch || 0; //in rads
   this.roll = conf.roll || 0; //in rads
   this.scale = conf.scale || new HPoint3D(1,1,1);
   this.focus = conf.focus || new HPoint3D(0,0,0);
   
   
   var rt=[]; //camera rotation + translation matrix
   for(var i=0; i<4; i++) {
      rt[i] = [];
      for(var j=0; j<4; j++) {
        rt[i][j] = undefined;
      }
   }
   
   this.calcRTMatrix = function () {
      rt[0][0] = Math.cos(this.roll) * Math.cos(this.pitch);
      rt[0][1] = (-Math.sin(this.roll) * Math.cos(this.yaw) + Math.cos(this.roll) * Math.sin(this.pitch) * Math.sin(this.yaw));
      rt[0][2] = (Math.sin(this.roll) * Math.sin(this.yaw) + Math.cos(this.roll) * Math.sin(this.pitch) * Math.cos(this.yaw));
      rt[0][3] = this.position.x;
      rt[1][0] = Math.sin(this.roll) * Math.cos(this.pitch);
      rt[1][1] = (Math.cos(this.roll) * Math.cos(this.yaw) + Math.sin(this.roll) * Math.sin(this.pitch) * Math.sin(this.yaw));
      rt[1][2] = (-Math.cos(this.roll) * Math.sin(this.yaw) + Math.sin(this.roll) * Math.sin(this.pitch) * Math.cos(this.yaw));
      rt[1][3] = this.position.y;
      rt[2][0] = Math.sin(this.pitch);
      rt[2][1] = Math.cos(this.pitch) * Math.sin(this.yaw);
      rt[2][2] = Math.cos(this.yaw) * Math.cos(this.pitch);
      rt[2][3] = this.position.z;
      rt[3][0] = 0;
      rt[3][1] = 0;
      rt[3][2] = 0;
      rt[3][3] = 1;
   };
   
   this.scalePoint3D = function (point){
      var p = new HPoint3D();
      p.x = this.scale.x * point.x;
      p.y = this.scale.y * point.y;
      p.z = this.scale.z * point.z;
      p.h = this.scale.h * point.h;
      return p;
   };
   
   this.getRTMatrix = function (){
      return rt;
   };
   
   this.applyRT2HPoint3D = function (point){
      var p = new HPoint3D();
      var rt = this.getRTMatrix();
   
      p.x = point.x * rt[0][0] + 
            point.y * rt[0][1] +
            point.z * rt[0][2] +
            point.h * rt[0][3];
   
      p.y = point.x * rt[1][0] + 
            point.y * rt[1][1] +
            point.z * rt[1][2] +
            point.h * rt[1][3];
   
      p.z = - point.x * rt[2][0] + 
            point.y * rt[2][1] +
            point.z * rt[2][2] +
            point.h * rt[2][3];
   
      p.h = point.x * rt[3][0] +
            point.y * rt[3][1] +
            point.z * rt[3][2] +
            point.h * rt[3][3];
   
      return p;

   };
   
   this.applyRT2CloudHPoint3D = function (points){
      var pointst =  [];
      for ( var i = 0; i < points.length; i ++ ) {
         pointst.push(this.applyRT2HPoint3D(points[i]));
      }
   
      return pointst;
   };
   
   this.calcRTMatrix();
   
   
   
}

/*
    * point2HPoint3D
    * convert a 3d point in the form of array to HPoint3D 
    * Params:
    *     - p (point3d in the form of array)
    */
function point2HPoint3D (p){
   return new HPoint3D(p[0],p[1],p[2]);
}

/*
    * cloudPoint2CloudHPoint3D
    * convert a 3d cloud point in the form of array to an array of HPoint3D 
    * Params:
    *     - cp (cloudpoint in the form of array)
    */
function cloudPoint2CloudHPoint3D (cp){
   var cp3d =  [];
   for ( var i = 0; i < cp.length; i += 3 ) {
      cp3d.push(new HPoint3D(cp[i],cp[i+1],cp[i+2]));
    }
   
   return cp3d;
}

/*
    * cloudPoint2CloudHPoint3D
    * convert an array of HPoint3D  to a 3d cloud point in the form of array
    * Params:
    *     - cp3d (array of HPoint3D)
    */
function cloudHPoint3D2CloudPoint (cp3d){
   var cp =  new Float32Array( cp3d.length * 3 );
   var j=0;
   for ( var i = 0; i < cp.length; i += 3 ) {
      
      cp[ i ]     = cp3d[j].x;
      cp[ i + 1 ] = cp3d[j].y;
      cp[ i + 2 ] = cp3d[j].z;
      
      j++;
    }
   
   return cp;

}



/*
    * transformHPoint3D
    * Applies the matrix rt to a HPoint3D
    * Params:
    *     - point (HPoint3D)
    *     - camera (TPinHoleCamera)
    */
function conicProjectionHPoint3D (point, camera){
   var a = new HPoint3D(0,0,0);
   var b = new HPoint3D(point.x,point.y,camera.focus.z);
   var c = new HPoint3D(0,0,point.z);
   var lamb = (c.z-a.z)/(b.z-a.z);
   c.x=a.x+lamb*(b.x-a.x);
   c.y=a.y+lamb*(b.y-a.y);
   
   var spoint= camera.scalePoint3D(c);
   return camera.applyRT2HPoint3D(spoint);

}

/*function conicProjectionHPoint3D (point, camera, d){
   var a = new HPoint3D(0,0,0);
   var b = point;
   var c = new HPoint3D(0,0,d);
   var lamb = (c.z-a.z)/(b.z-a.z);
   c.x=a.x+lamb*(b.x-a.x);
   c.y=a.y+lamb*(b.y-a.y);
   
   var spoint= camera.scalePoint3D(c);
   return camera.applyRT2HPoint3D(spoint);

}*/

/*
    * transformHPoint3D
    * Applies scale and the matrix rt to a HPoint3D
    * Params:
    *     - point (HPoint3D)
    *     - camera (TPinHoleCamera)
    */
function scalarProjectionHPoint3D (point, camera){
   var spoint= camera.scalePoint3D(point);
  
   return camera.applyRT2HPoint3D(spoint);

}
/*function transformHPoint3D (point, camera){
   var p = new HPoint3D();
   
   p.x = point.x * Math.cos(camera.roll) * Math.cos(camera.pitch) + 
           point.y * (-Math.sin(camera.roll) * Math.cos(camera.yaw) + Math.cos(camera.roll) * Math.sin(camera.pitch) * Math.sin(camera.yaw)) +
           point.z * (Math.sin(camera.roll) * Math.sin(camera.yaw) + Math.cos(camera.roll) * Math.sin(camera.pitch) * Math.cos(camera.yaw)) +
           point.h * camera.position.x;
   
   p.y = point.x * Math.sin(camera.roll) * Math.cos(camera.pitch) + 
           point.y * (Math.cos(camera.roll) * Math.cos(camera.yaw) + Math.sin(camera.roll) * Math.sin(camera.pitch) * Math.sin(camera.yaw)) +
           point.z * (-Math.cos(camera.roll) * Math.sin(camera.yaw) + Math.sin(camera.roll) * Math.sin(camera.pitch) * Math.cos(camera.yaw)) +
           point.h * camera.position.y;
   
   p.z = - point.x * Math.sin(camera.pitch) + 
           point.y * Math.cos(camera.pitch) * Math.sin(camera.yaw) +
           point.z * Math.cos(camera.yaw) * Math.cos(camera.pitch) +
           point.h * camera.position.z;
   
   p.h = point.x * 0 +
         point.y * 0 +
         point.z * 0 +
         point.h * 1;
   
   return p;

}*/

/*
    * transformCloudHPoint3D
    * Applies the matrix rt to an array of HPoint3D
    * Params:
    *     - points (array of HPoint3D)
    *     - camera (TPinHoleCamera)
    */
function scalarProjectionCloudHPoint3D (points,camera){
   var pointst =  [];
   for ( var i = 0; i < points.length; i ++ ) {
      pointst.push(scalarProjectionHPoint3D(points[i],camera));
    }
   
   return pointst;
}

/*
    * transformCloudHPoint3D
    * Applies the matrix rt to an array of HPoint3D
    * Params:
    *     - points (array of HPoint3D)
    *     - camera (TPinHoleCamera)
    */
function conicProjectionCloudHPoint3D (points,camera){
   var pointst =  [];
   for ( var i = 0; i < points.length; i ++ ) {
      pointst.push(conicProjectionHPoint3D(points[i],camera));
    }
   
   return pointst;
}

//function conicProjectionCloudHPoint3D (points,camera,dist){
//   var pointst =  [];
//   for ( var i = 0; i < points.length; i ++ ) {
//      pointst.push(conicProjectionHPoint3D(points[i],camera,dist[i]));
//    }
//   
//   return pointst;
//}



