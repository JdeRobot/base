var API = API || {};
/*
 * API.CmdVel's Constructor.
 * Params:
 *     - config (contents client's Config)={server,id,epname}:
 *       + server (server's direction and port)={dir:direction,port: port}
 *       + epname (name of cmdvel endpoint, default CmdVel)
 */
API.CmdVel= function (config){
   var conf = config || {};
   
   this.workerFile = "js/libs/cmdvel/cmdvel_worker.js";
   this.w = undefined; 
   this.isRunning = false;
   
   this.conPromise = undefined;
   
   
   this.epname = conf.epname || "CmdVel";
   this.server = conf.server;
      
   var self = this;
   
   
   this.createWork = function(){
      this.w = new Worker(this.workerFile);
      this.w.onerror = function (err){
         console.log ("worker error: "+ err.message);
      };
   };
   
   this.deleteWork = function (){
      if (this.w){
         this.w.terminate();
         this.w = undefined;
      }
      if(self.toError){
         clearTimeout(self.toError);
         toError=undefined;
      }
   };
   
   this.connect = function (){
      this.conPromise = new Promise(
        // The resolver function is called with the ability to resolve or
        // reject the promise
        function(resolve, reject) {
            self.w.onmessage = function(mes){
               if (mes.data){
                  self.isRunning = true;
                  resolve();
               }else{
                  self.isRunning = false;
                  reject();
               }
            };
            self.w.postMessage({func:"connect",serv:self.server,epname: self.epname}); 
        });
     
   };
   
   this.disconnect = function (){
      this.w.postMessage({func:"disconnect"});
      this.isRunning = false;
      if(self.toError){
         clearTimeout(self.toError);
         toError=undefined;
      }
   };
   /*
   vel= {
         linearX,
		 linearY,
		 linearZ,
		 angularX,
		 angularY,
		 angularZ		
   };
   */
   this.setCmdVel = function(vel){
      this.conPromise.then(
        function() {
            self.w.postMessage({func:"setCMDVelData",vel:vel});
        }); 
   };
   
   
   
   this.createWork();
   
};