var ros = new ROSLIB.Ros({
	url : 'ws://192.168.1.12:9090'
});

var navigation = false

$(window).resize(function(){
	location.reload();
	}
);

//jquery init
$(document).ready(function() {

	$('div#nav').width('70%');/* $( "#urdf" ).width(); */

	$('div#panel').width('25%');

	var anchodiv =  $('div#tabs-1').width();

	var anchonav = $("#nav").width();

	var altonav = anchonav / 1.33;

	var anchopanel = anchodiv - anchonav - 1550;

	$('div#tabs-1').height(altonav);

	//$('div#panel').width(anchopanel);


	//$('div#tabs-1').width(ancho);

	// Create the main viewer.
	var viewer = new ROS2D.Viewer({
	  divID : 'nav',
	  width : anchonav,
	  height : altonav - 250,			
	});
	// Setup the nav client.
	var navGridClient = new NAV2D.OccupancyGridClientNav({
	ros : ros,
	rootObject : viewer.scene,
	viewer : viewer,
	withOrientation: true,
	serverName : '/move_base'
    });
	
});

function startNavigation(){

	if(navigation == true){
		window.alert("Error: Nodos AMCL y move_base iniciados");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : '/map_nav_manager_node/start_navigation_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});	
	navigation = true;
	window.alert("Nodos AMCL y move_base se han iniciado. Presione OK.");
	
	}

}

function stopNavigation(){

	if(navigation == false){
		window.alert("Error: Nodos AMCL y move_base no iniciados");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : '/map_nav_manager_node/stop_navigation_srv',
		messageType : 'std_srv/Trigger'
	});

	var data = new ROSLIB.ServiceRequest({
		value : ''
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});
	navigation = false;
	window.alert("Nodos AMCL y move_base se han parado. Presione OK.");
	
	}
}

function loadMap(){

	if(navigation==true){

		window.alert("Error. Nodos AMCL y move_base se han iniciado");

	}else{

		var file_name = $('#filename').val();

		if(file_name == ''){
			window.alert("Error. Escriba un nombre de archivo.");
		}else{

			console.log(file_name);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name : '/map_nav_manager_node/load_map_srv',
				messageType : 'std_srv/Trigger'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name
			});

			svc.callService(data,function(res){
				console.log("Respuesta Recibida");
			});	

			window.alert("El mapa se ha cargado con \u00e9xito.");
		}
	}

}

function goIndex(){
	if(navigation==true){
		stopNavigation();
	}
    window.location.href = "index.html";
}

function goMapping(){
	if(navigation==true){
		stopNavigation();
	}
    window.location.href = "mapping.html";
}
