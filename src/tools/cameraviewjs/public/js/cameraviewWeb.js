var config={};
$(document).ready(function() {
   
   var camera;
   load();
   config.camid= "camView";
   config.fpsid= "fps";
   config.sizeid= "size";
   
   $('#start').on('click', function(){
      $("#camView").removeClass("border-light");
		 camera = new CameraView(config);
         camera.start();
	});
   
   $('#stop').on('click', function(){
         camera.stop();
	});
   
   var resize = function (){
      $(".cam").height( $(".cam").width()*3/4);
   };
   
   $(window).resize(function(){
      resize();
   });
   
   $('#save').on('click', function(){
      config.serv.dir= $('#dir').val();
      config.serv.port= $('#port').val();
      config.epname= $('#ep').val();
      localStorage["cameraviewconfig"]=JSON.stringify(config);
	});
   
   resize();
});


function load(){
   if (localStorage.getItem("cameraviewconfig")) {
       config = JSON.parse(localStorage.getItem("cameraviewconfig"));
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