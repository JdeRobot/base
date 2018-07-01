let config = {};
var w;
var lineInterval, pointInterval, posInterval, objInterval;
var cont = 1;
class obj3DPose {
	constructor(id,x,y,z,rx,ry,rz){
		this.id = id;
		this.x = x;
		this.y = y;
		this.z = z;
		this.rx = rx;
		this.ry = ry;
		this.rz = rz;
	}
}

try{
	const yaml = require('js-yaml');
	const fs = require('fs');
	config = yaml.safeLoad(fs.readFileSync('public/config.yml', 'utf8'))
} catch (e) {
	config.Server = "localhost";
	config.Port = "11000";
	config.updatePoints= 10
	config.updateSegments= 10
	config.linewidth= 2
	config.pointsize= 8
	config.camera = {}
	config.camera.x = 100
	config.camera.y = 50
	config.camera.z = 300

}
    function startWorker(){
      if(typeof(Worker) !== "undefined") {
        if(typeof(w) == "undefined") {
          w = new Worker("js/3DViz_worker.js");
          w.postMessage({func:"Start",server:config.Server, port:config.Port});
      }
      } else {
        Console.log("Sorry, your browser does not support Web Workers...");
      }
      w.onmessage = function(event) {
        if (event.data.func == "Connect"){
					setPlane();
        } else {
          console.log(event.data);
          w.terminate();
        }
      }
    }

    function stop(){
      if(typeof(w) != "undefined") {
        w.postMessage({func:"Stop"});
        w.onmessage = function(event) {
					if (event.data.func == "Disconnect"){
            w.terminate();
            w = undefined;
      }
    }}
    }

		function setPlane(){
			w.postMessage({func:"setLine"});
			w.onmessage = function(event){
				if (event.data.segments.buffer.length > 0){
						segments = event.data.segments.buffer;
						for (var i = 0; i < segments.length; i+=1) {
		        	addLine(segments[i], "plane");
						}
			} pointInterval = setInterval(function(){
				setPoint();
			},config.updatePoints);
			lineInterval = setInterval(function(){
				setLine();
			},config.updateSegments);
			objInterval = setInterval(function(){setObj();},1000);}
		}

		function setLine(){
			w.postMessage({func:"setLine"});
			getData();
		}

    function setPoint(){
      w.postMessage({func:"setPoint"});
			getData();
		}
		function setObj(){
			id = "obj" + cont;
			w.postMessage({func:"setObj", id: id});
			getData();
		}

		function setPose3D(){
			w.postMessage({func:"setPose3D"});
			getData();
		}

		function getData (){
			w.onmessage = function(event) {
				if (event.data.func == "drawLine"){
					if (event.data.segments.refresh & (event.data.segments.buffer.length !=0)){
						deleteObj("segments");
					}
						segments = event.data.segments.buffer;
						for (var i = 0; i < segments.length; i+=1) {
		        	addLine(segments[i], "segments");
						}
				} else if (event.data.func == "drawPoint"){
					if (event.data.points.refresh & (event.data.points.buffer.length != 0)){
						deleteObj("points");
					}
					points = event.data.points.buffer;
				for (var i = 0; i < points.length; i+=1) {
        	addPoint(points[i]);
				}
			} else if (event.data.func == "drawObj") {
				cont += 1
				addObj(event.data.obj);
				posInterval = setInterval(function(){
					setPose3D();
				}, 5000);

			} else if (event.data.func == "pose3d") {
				for (var i = 0; i < event.data.bpose3d.length; i += 1){
					data = event.data.bpose3d[i];
					console.log(data);
					rotateZ=getYaw(data.pos.q0,data.pos.q1,data.pos.q2,data.pos.q3);
        	rotateY=getPitch(data.pos.q0,data.pos.q1,data.pos.q2,data.pos.q3);
        	rotateX=getRoll(data.pos.q0,data.pos.q1,data.pos.q2,data.pos.q3);
					objpose3d = new obj3DPose(data.id,data.pos.x,data.pos.y,data.pos.z,rotateX,rotateY,rotateZ);
					moveObj(objpose3d);}
			}
			}
		}
