var API = API || {};
/*
 * API.Motors's Constructor.
 * Params:
 *     - config (contents client's Config)={server,epname,onmessage}:
 *       + server (server's direction and port)={dir:direction,port: port}
 *       + epname (name of motor endpoint, default Motors)
 */
API.Motors = function (config){
   var conf = config || {};
   
   this.id = conf.id;
   
   this.workerFile = "js/libs/motors/motors_worker.js";
   this.w = undefined; 
   this.onmessage = undefined;
   this.isRunning = false;
   
   this.conPromise = undefined;
   
   this.data=undefined;
   
   this.epname = conf.epname || "Motors";
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
   
   
   
   this.getAll = function(){
      this.conPromise.then(
         function() {
            self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func: "getAll"});
         });
   };
   
   this.getV = function(){
      this.conPromise.then(
         function() {
            self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func: "getV"});
         });
   };
   
   this.getW = function(){
      this.conPromise.then(
         function() {
            self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func: "getW"});
         });
   };
   
   this.getL = function(){
      this.conPromise.then(
         function() {
            self.w.onmessage = self.onmessage || self.onmessageDefault;
            self.w.postMessage({func: "getL"});
         });
   };
   
   this.setAll = function(v,w,l){
      this.conPromise.then(
         function() {
            var data = {v:v,w:w,l:l};
            self.w.postMessage({func: "setAll", data: data});
         });
   };
   
   this.setV = function(v){
      this.conPromise.then(
         function() {
            self.w.postMessage({func: "setV", data: v});
         });
   };
   
   this.setW = function(w){
      this.conPromise.then(
         function() {
            self.w.postMessage({func: "setW", data: w});
         });
   };
   
   this.setL = function(l){
      this.conPromise.then(
         function() {
            self.w.postMessage({func: "setL", data: l});
         });
   };
   
     
   this.onmessageDefault = function(event) {
         self.data=event.data;
      };

   this.createWork();
};