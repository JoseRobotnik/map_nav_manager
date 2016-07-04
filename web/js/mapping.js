var ros = new ROSLIB.Ros({
	url : 'ws://192.168.1.12:9090'
});

var mapping = false

//jquery init
$(document).ready(function() {

	$('div#occupancygrid').width('70%');/* $( "#urdf" ).width(); */

	$('div#panel').width('25%');

	var anchodiv =  $('div#tabs-1').width();
	
	var anchonav = $('div#occupancygrid').width();

	var altonav = anchonav / 1.33;

	var anchopanel = anchodiv - anchonav - 1550;

	$('div#tabs-1').height(altonav);

	$(window).resize(function(){
		location.reload();
		}
	);

	//$('div#panel').width(anchopanel);


	//$('div#tabs-1').width(ancho);
	
	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
  		divID : 'occupancygrid',
		width : anchonav,
		height : altonav - 250,
		antialias : true,
		background: '#002233',
		cameraPose : {x: 3, y: 3, z: 1}
	});

	// Add a grid.
	viewer.addObject(new ROS3D.Grid({
		color:'#0181c4',
  		cellSize: 0.5,
  		num_cells: 20
	}));

	// Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      fixedFrame : '/map',
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene,
      loader : ROS3D.COLLADA_LOADER
    });
    
    // Setup the marker client.
    var gridClient = new ROS3D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene,
      continuous: true
    });
	
});

function startMapping(){

	if(mapping == true){
		window.alert("Error: Nodo SLAM-Gmapping iniciado");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : '/map_nav_manager_node/start_mapping_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});	
	mapping = true;
	window.alert("El nodo SLAM-Gmapping se ha iniciado. Presione OK.");
	
	}

}

function stopMapping(){

	if(mapping == false){
		window.alert("Error: Nodo SLAM-Gmapping no iniciado");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : '/map_nav_manager_node/stop_mapping_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});
	mapping = false;
	window.alert("El nodo SLAM-Gmapping se ha parado. Presione OK.");
	
	}
}

function saveMap(){

	if(mapping==false){

		window.alert("Error. Nodo SLAM-Gmapping no iniciado");

	}else{

		var file_name = $('#filename').val();

		if(file_name == ''){
			window.alert("Error. Escriba un nombre de archivo.");
		}else{

			console.log(file_name);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name : '/map_nav_manager_node/save_map_srv',
				messageType : 'map_nav_manager/SetFilename'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name
			});

			svc.callService(data,function(res){
				console.log("Respuesta Recibida");
			});	

			window.alert("El mapa se ha guardado con \u00e9xito.");
		}
	}

}

function goIndex(){
	if(mapping==true){
		stopMapping();
	}
    window.location.href = "index.html";
}

function goNavigation(){
	if(mapping==true){
		stopMapping();
	}
    window.location.href = "navigation.html";
}