var API = API || {};
/*
 * API.ArDroneExtra's Constructor.
 * Params:
 *     - config (contents client's Config)={server,id,epname}:
 *       + server (server's direction and port)={dir:direction,port: port}
 *       + epname (name of pose3d endpoint, default ArDroneExtra)
 */
API.ArDroneExtra= function (config){
   var conf = config || {};
   this.workerFile = "js/libs/ardroneextra/ardroneextra_worker.js";
   this.w = undefined; 
   this.onmessage = undefined;
   this.isRunning = false;
   
   this.conPromise = undefined;
   
   this.data = undefined;
   this.update = false;
   
   this.epname = conf.epname || "Extra";
   this.server = conf.server;
   
   this.toError = undefined; // connection error message timeout
   this.timeoutE = 3000;
   
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
   
   this.toggleCam = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"toggleCam"});
        }); 
   };
   
   this.land = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"land"});
        }); 
   };
   
   this.takeoff = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"takeoff"});
        }); 
   };
   
   this.reset = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"reset"});
        }); 
   };
   
   this.recordOnUsb = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"recordOnUsb"});
        }); 
   };
   
   this.ledAnimation = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"ledAnimation"});
        }); 
   };
   
   this.flightAnimation = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"flightAnimation"});
        }); 
   };
   
   this.flatTrim = function(){
      this.conPromise.then(
        function() {
           self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func:"flatTrim"});
        }); 
   };
      
   
   this.onmessageDefault = function(event) {
      if (self.toError){
         clearTimeout(this.toError);
         self.toError=setTimeout(self.conErr, self.timeoutE);
      }
      var respwork=event.data;
      self.data= respwork;
      };
   /*
    * conErr
    * Displays an error message and closes the webworker
    */
   this.conErr=function(){
      alert ("connection to server "+self.epname+" "+self.server.dir+":"+self.server.port+" failed");
      self.deleteWork();
   };
   
   this.createWork();
   
};