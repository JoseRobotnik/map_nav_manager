  /**
   * Setup all visualization elements when the page is loaded. 
   */
  function init() {
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://192.168.1.12:9090'
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'viewer_grid',
      width : 800,
      height : 600,
      antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
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
    
     // Create the main viewer.
    var viewer2D = new ROS2D.Viewer({
      divID : 'nav',
      width : 350,
      height : 400
    });
    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer2D.scene,
      viewer : viewer2D,
      serverName : '/move_base',
      image: 'turtlebot.png'
    });
  }