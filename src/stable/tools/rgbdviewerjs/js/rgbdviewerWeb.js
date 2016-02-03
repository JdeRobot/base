var config={};
$(document).ready(function() {
   var viewer;
   load();
   config.camid= "camView";
   config.fpscamid= "fps";
   config.depthid= "camView2";
   config.fpsdepid= "fps2";
   config.modelid= "model";
   
   $('#start').on('click', function(){
      viewer = new RgbdViewer(config);
      viewer.start();
       $("canvas.border-light").removeClass("border-light");
	});
   $('#stop').on('click', function(){
         viewer.stop();
	});
   
   
   var resize = function (){
      $(".cam").height( $(".cam").width()*3/4);
   };
   
   
   $(window).resize(resize);
   
   $('#save').on('click', function(){
      config.serv.dir= $('#dir').val();
      config.serv.port= $('#port').val();
      config.camepname= $('#eprgb').val();
      config.depepname= $('#epdepth').val();
      localStorage["rgbdviewerconfig"]=JSON.stringify(config);
	});
   
   resize();
});

function load(){
   if (localStorage.getItem("rgbdviewerconfig")) {
       config = JSON.parse(localStorage.getItem("rgbdviewerconfig"));
      $('#dir').val(config.serv.dir);
      $('#port').val(config.serv.port);
      $('#ep').val(config.epname);
     
    } else{
      config.serv={};
      config.serv.dir= $('#dir').val();
      config.serv.port= $('#port').val();
      config.epname= $('#ep').val();
    }
}