// variables to make believe to ICE that is in the main thread
var window=self;
var global=self;

// importing required files
importScripts('../Ice.min.js');
importScripts('../jderobot/pose3dencoders.js');


// variables related to the configuration and connection of ICE
var id = new Ice.InitializationData();
var communicator;
var connection;

var Promise = Ice.Promise;
var Prx = jderobot.Pose3DEncodersPrx;
var str = undefined;


function getPose3D(config){

   try {
    // Create the proxy to connect
    var proxy = communicator.stringToProxy(config.epname+":ws -h " + config.server.dir + " -p " + config.server.port);
       Prx.checkedCast(proxy).then(function(server){
         server.getPose3DEncodersData().then(function (data){
                var response=data;
                postMessage(response);
                
        },function(err){console.log(err);});
    }).exception(
    function(ex) {
        console.log("error:"+err);
        var ee= new Error("no se pudo conectar con el servidor");
    console.log(ee);
    throw ee;
    });
        
    }catch(err) {
        console.log("error 12:"+err);
    }
}


onmessage = function(e) {
    //console.log("camera worker");
    var config={};
    // Collects the data provided by the main thread and initialized ICE
    config.server = e.data.serv;
    config.epname = e.data.epname;
   
    id.properties = Ice.createProperties();
    communicator = Ice.initialize();
    switch (e.data.func){
      case "getPose3D":
         getPose3D(config);
         break;
      case "startPose3DStream":
         str = setInterval(getPose3D,80,config);
         break;
      case "stopPose3DStream":
         if (str){
               clearInterval(str);
            }
         break;
      default:
          
    } 
}