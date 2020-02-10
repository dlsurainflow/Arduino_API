jQuery(document).ready(main)
function main() {
	var window_height = $(window).height(),
      header_height = $(".main-header").height();
$("#myDiv").css("height", window_height - header_height);

	var map = L.map('map');
        map.setView([14.6047, 120.97833], 18);

        L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1IjoibWFwYm94IiwiYSI6ImNpejY4NXVycTA2emYycXBndHRqcmZ3N3gifQ.rJcFIG214AriISLbB6B5aw', {
		maxZoom: 20,
		attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, ' +
			'<a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ' +
			'Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
		id: 'mapbox/streets-v11'
	}).addTo(map);

	var LOCpointer = L.icon({
    iconUrl: '5-5-01.png',

    iconSize: [30, 43], // size of the icon
    shadowSize: [50, 64], // size of the shadow
    iconAnchor: [22, 94], // point of the icon which will correspond to marker's location
    popupAnchor: [-7, -95]
  });

		var markerHome = L.marker([14.6047, 120.97833], { icon: LOCpointer }).addTo(map).on('click', function () {
            sidebar.show();
			sidebar.setContent("<h2><b>Johann's RAFT Node!</b></h2><p>RAFT shows Flood Level at <b>XXXX</b></p><p>RAFT shows Rain Intensity at <b>YYYY</b></p>");
        });
		
		var markerSomewhere = L.marker([14.65, 120.97833], { icon: LOCpointer }).addTo(map).on('click', function () {
            sidebar.show(); 
			sidebar.setContent("<h2><b>Somewhere's RAFT Node!</b></h2><p>RAFT shows Flood Level at <b>XXXX</b></p><p>RAFT shows Rain Intensity at <b>YYYY</b></p>");
        });
		
		markerHome.bindPopup("<center><b>RAFT Here!</b><br>Decriptive Flood Level <br>Decriptive Rain Intensity</center>");
		markerSomewhere.bindPopup("<center><b>RAFT Here!</b><br>Decriptive Flood Level <br>Decriptive Rain Intensity</center>");

        var sidebar = L.control.sidebar('sidebar', {
            closeButton: true,
            position: 'right'
        });
        map.addControl(sidebar);

        setTimeout(function () {
            sidebar.hide();
        }, 500);

        

        map.on('click', function (e) {
            sidebar.toggle();
			sidebar.setContent("You clicked the map at " + e.latlng.toString() +"EWW");
        })

        sidebar.on('show', function () {
            console.log('Sidebar will be visible.');
        });

        sidebar.on('shown', function () {
            console.log('Sidebar is visible.');
        });

        sidebar.on('hide', function () {
            console.log('Sidebar will be hidden.');
        });

        sidebar.on('hidden', function () {
            console.log('Sidebar is hidden.');
        });

        L.DomEvent.on(sidebar.getCloseButton(), 'click', function () {
            console.log('Close button clicked.');
        });
	
  
}

