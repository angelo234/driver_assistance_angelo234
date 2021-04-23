angular.module('beamng.apps')
.directive('driverAssistanceAngelo234', ['$log', 'StreamsManager', 'Utils', function ($log, StreamsManager, Utils) {
  return {
    restrict: 'E',
    template:
    `<div style="height:100%; font-family:Consolas,Monaco,Lucida Console,Liberation Mono;">
		<table id="table" style="width: 100%;">
			<tbody id="tableBody">		
				<tr>	
	  
					<div layout="column" style="position: absolute; top: 10px; left: 5px; font-weight:bold;">
						<small ng-repeat="graph in graphList" style="color:{{ graph.color }}; padding:2px">{{ graph.title | translate }}</small>
					</div>
					<canvas width="400" height="200">	
				</tr>
				<tr>
					<md-button ng-click="pauseResumeGraph()" class="md-raised md-primary" flexmd-no-ink>Pause/Resume</md-button>
				</tr>
			</tbody>
		</table>
    </div>

	`,

    replace: true,
    link: function (scope, element, attrs) {
      var streamsList = ['wheelInfo', 'electrics', 'sensors'];
      StreamsManager.add(streamsList);
      scope.$on('$destroy', function () {
        StreamsManager.remove(streamsList);
      });

      colors = [];
      for (var i=15; i>0; i--) {
        var c = Utils.rainbow(15, i);
        colors.push(`rgb(${Math.round(255 * c[0])}, ${Math.round(255 * c[1])}, ${Math.round(255 * c[2])})`);
      }

		var paused = false;

      var graphs = {};
      scope.graphList = [
	  {title: 'Throttle', color: colors[0]},
	  {title: 'Acceleration', color: colors[4]},	  
	  ];
	
      var canvas = element[0].getElementsByTagName('canvas')[0];

      var chart = new SmoothieChart({
          minValue: 0.0,
          // maxValue: 1500,
          millisPerPixel: 20,
          interpolation: 'linear',
          grid: { fillStyle: 'rgba(255, 255, 255, 1)', strokeStyle:'rgba(0,0,0,0.96)', verticalSections: 6, millisPerLine: 1000, sharpLines: true },
          labels: {fillStyle: 'black'}
        })
        , throttle_graph = new TimeSeries()
		, acc_graph = new TimeSeries()

      var globalMax = 1;

      chart.addTimeSeries(throttle_graph,   {strokeStyle:colors[1], lineWidth:3});  
      chart.addTimeSeries(acc_graph,   {strokeStyle: colors[4], lineWidth: 3});  
	  
	  chart.streamTo(canvas, 0);

      scope.$on('streamsUpdate', function (event, streams) {
		var xPoint = new Date();
		throttle_graph.append(xPoint, streams.electrics.throttle);
		globalMax = Math.max(globalMax, streams.electrics.throttle);
		
		acc_graph.append(xPoint, -streams.sensors.gy2 / 9.81);
		globalMax = Math.max(globalMax, -streams.sensors.gy2 / 9.81);

		chart.options.maxValue = globalMax;
		  
		if(streams.sensors.gy2 != 0){
			//console.log("hello")
		}	
      });
	  
	  scope.pauseResumeGraph = function(){
		  if(paused){
			  chart.start();
		  }
		  else{
			  chart.stop();
		  }
		  
		  paused = !paused
		};

      scope.$on('VehicleReset', function (event, data) {
        graphs = {};
        scope.graphList = [{title: 'Speed', color: colors[0]}];
        scope.$digest();
      });

      scope.$on('VehicleChange', function (event, data) {
        graphs = {};
        scope.graphList = [{title: 'Speed', color: colors[0]}];
        scope.$digest();
      });

      scope.$on('app:resized', function (event, data) {
        //canvas.width = data.width;
        //canvas.height = data.height;
      });
    }
  };
}]);
