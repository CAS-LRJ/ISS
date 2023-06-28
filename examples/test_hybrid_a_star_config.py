## Config File
import math

config = {
    'Road': {
        ## Road Parameters
        'RoadMaxAngle': math.pi / 180. * 5.
    },
    'Sensors': {
        ## Lidar Parameters
        'LidarShiftX': 0.,
        'LidarShiftY': 0.,
        'LidarShiftZ': 0.
    },
    'Vehicle': {
        ## Vehicle Parameters
        'VehicleHeight': 2.5,
        'VehicleLength': 4.4,
        'VehicleWidth': 2.2,
        'MaxSteer': math.pi / 5.,
        'TurningRadius': 5.,
        'VehicleXShift': 0.,
        'VehicleYShift': 0.,
        'VehicleZShift': 0.
    },
    'Perception': {        
    },
    'Localization' : {    
    },
    'Orientation': {    
    },
    'Mapping': {
        ## Fast Lidar Mapping Parameters
        'FLM_DetectRadius': 16,
        'FLM_Stepsize': 0.1,
        'FLM_Tolerance': 0.05,
        'FLM_VehicleHeight':2.5
    },
    'Planning': {        
        ## Dynamic Progamming Parameters
        'DP_SoftPenalty': 100.,
        'DP_Stepsize': 0.5,
        'DP_ObstacleDetect': 1.1,
        'DP_ObstacleSearchStepsize': 64,
        ## Hybrid A Star Parameters
        'HA_NSteer': 11,
        'HA_SoftCollisionCost': 100.,
        'HA_SteerChangeCostCoeff': 5.0,
        'HA_SteerCostCoeff': 1.0,
        'HA_HeuristicCostCoeff': 5,
        'HA_Stepsize': 1.5,
        'HA_YawGridSize': math.pi / 36.,
        'HA_XYGridSize': 1.0,
        'HA_RouteInterval': 1.0,
    },
    'Control': {
        ## PID Parameters        
    }
}