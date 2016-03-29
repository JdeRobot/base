var GUI = GUI || {};

GUI.Joystick = function (config){
   var conf = config || {};
   
   this.id = conf.id;
   //this.onDown = conf.onDown || function (){};
   this.onMove = conf.onMove || function (){};
   this.onUp = conf.onUp || function (){};
   
   var self = this;
   var canvas = document.getElementById(this.id);
   var ctx = canvas.getContext("2d");
   // opacidad 
   ctx.globalAlpha = 0.85;
   var cw = canvas.width;
   var ch = canvas.height;
   console.log("cw:"+cw+" ch: "+ch);
   var X = cw / 2;
   var Y = ch / 2;
   var lineWidth = 1;
   ctx.lineWidth = lineWidth;
   var RHoop = X * 0.7; //Diametro de aro
   var RCircle = RHoop * 0.3; // Diametro del circulo
   var maxR = RHoop; // Distancia maxima a la que puede moverse
        
   var arrastrar = false;
   
   var p = {
         'X': X,
         'Y': Y,
         'R': RCircle
   };
   var delta = {
         'x': X,
         'y': Y,
         'r': RCircle};
   
   function oMousePos(canvas, evt) {
      // Detecta la posición del raton en un canvas
      var ClientRect = canvas.getBoundingClientRect();
      return { //objeto
         x: Math.round(evt.clientX - ClientRect.left),
         y: Math.round(evt.clientY - ClientRect.top)
      };
   }

   // Posicion si tocamos la pantalla
   function getTouchPos(canvasDom, touchEvent) {
      var rect = canvasDom.getBoundingClientRect();
      console.log(touchEvent);
        return {
                x: touchEvent.targetTouches[0].clientX - rect.left,
                y: touchEvent.targetTouches[0].clientY - rect.top
        };
   }
   
   function dibujarAro(x, y, r) {
      ctx.beginPath();
      ctx.arc(x, y, r, 0, 2 * Math.PI, true);
      ctx.strokeStyle = "rgb(88, 217, 255)";
      ctx.stroke();
   }

   function dibujarCirculo(x, y, r) {
      ctx.beginPath();
      ctx.fillStyle = "rgb(88, 217, 255)";
      ctx.arc(x, y, r, 0, 2 * Math.PI, true);
      ctx.fill();
   }
   
   function move (m){
      delta.x = m.x - p.X;
      delta.y = m.y - p.Y;
      var deltaR = Math.sqrt(delta.x * delta.x + delta.y * delta.y);
      var elR = Math.min(deltaR, maxR);
      //console.log("DeltaR: " + deltaR + " elR: " + elR);
      var angulo = Math.atan2(delta.y, delta.x);
      //console.log(angulo); //

      var x0 = X + elR * Math.cos(angulo);
      var y0 = Y + elR * Math.sin(angulo);

      var x = (x0 - X)/maxR;
      var y = (y0 - Y)/maxR*(-1);
      
      ctx.clearRect(0, 0, cw, ch); // Clear and redraw the joystick
      dibujarAro(X, Y, RHoop);
      dibujarCirculo(x0, y0, RCircle);
      
      self.onMove(x,y);
   }
   
   //listeners
   
   function onMouseDown (evt){
      var mousePos = oMousePos(canvas, evt);

      if (ctx.isPointInPath(mousePos.x, mousePos.y)) {
         arrastrar = true;
      }
   }
   
   function onTouchStart (evt){
      var mousePos = getTouchPos(canvas, evt);

      if (ctx.isPointInPath(mousePos.x, mousePos.y)) {
         arrastrar = true;
      }
   }
   
   function onMouseMove (evt){
      var m = oMousePos(canvas, evt);
      
      if (arrastrar) {
         move(m);
      }
   }
   
   function onTouchMove (evt){
      var m = getTouchPos(canvas, evt);
      
      if (arrastrar) {
         move(m);
      }
   }
   
   function onEnd (evt){
      arrastrar = false;
      ctx.clearRect(0, 0, cw, ch);
      dibujarAro(X, Y, RHoop);
      dibujarCirculo(X, Y, RCircle);
      
      self.onUp(0,0);
   }
   
   function preventDefault(e){
      if (e.target == canvas) {
         e.preventDefault();
      }
   }
   
   // dibujamos los dos compoentes del joystick
   dibujarAro(X, Y, RHoop);
   dibujarCirculo(p.X, p.Y, RCircle);
   
   // agregamos los listeners
   canvas.addEventListener('mousedown', onMouseDown, false);
   canvas.addEventListener('mousemove', onMouseMove, false);
   canvas.addEventListener('mouseup', onEnd, false);
   canvas.addEventListener('mouseout', onEnd, false);
   
   canvas.addEventListener('touchstart', onTouchStart, false);
   canvas.addEventListener('touchmove', onTouchMove, false);
   canvas.addEventListener('touchend', onEnd, false);
   canvas.addEventListener('touchup', onEnd, false);
   
   //document.body.addEventListener("touchstart", preventDefault, false);
   //document.body.addEventListener("touchend", preventDefault, false);
   //document.body.addEventListener("touchmove", preventDefault, false);

};