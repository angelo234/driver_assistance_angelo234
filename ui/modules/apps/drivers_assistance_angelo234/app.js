angular.module('beamng.apps')

.constant('UI_TEXT', {

	surfaces: [
		{ surface: 'ALL_SURFACES',  	txt: 'All Surfaces'		},
	]
})

.directive('parkingAidAngelo234', ['UI_TEXT', 'bngApi', function (UI_TEXT, bngApi) {
return {
templateUrl: 'modules/apps/drivers_assistance_angelo234/app.html',
replace: true,
restrict: 'EA',
link: function (scope, element, attrs) {
	
	//FUNCTIONS
	
	function init(){
		// The current overlay screen the user is on (default: null)
		scope.overlayScreen = null;	

	}

	//START

	init();


	// An optional list of streams that will be used in the app
	//var streamsList = [/* streams here */];

	// Make the needed streams available.
	//StreamsManager.add(streamsList);

	// Make sure we clean up after closing the app.
	scope.$on('$destroy', function () {
		//StreamsManager.remove(streamsList);
	});

	scope.$on('streamsUpdate', function (event, streams) {
	/* Some code that uses the streams' values */
	});
},
};
}]);