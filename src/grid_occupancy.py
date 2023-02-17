import math
import string
from sys import gettrace
from tkinter import Grid
from matplotlib.pyplot import grid
import numpy as np
from typing import Callable, Dict, Tuple, List
import time



class GridSize():
    def __init__(self, width_meters: float, height_meters: float) -> None:
        self.width = width_meters
        self.height = height_meters

class DoorGridSize():
    def __init__(self, grid_size:GridSize, low_zone_height: float, high_zone_height: float) -> None:
        self.width = grid_size.width
        self.height = grid_size.height
        self.low_zone_height = low_zone_height
        self.high_zone_height = high_zone_height

class ZoneOfInterest():
    def __init__(self, x:float, y:float, width:float, height:float, id:string) -> None:
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.id = id

    def contains(self, x:float, y:float) -> bool:
        '''Returns true if the position (x,y) is inside the zone'''
        if (x>=self.x) and (x<=self.x+self.width):
            if (y>=y) and (y<self.y+self.height):
                return True
        
        return False


class OccupancyGrid():
    CELL_SIZE = 0.2

    def __init__(self, width_meters: float, height_meters: float) -> None:
        self.width = width_meters
        self.height = height_meters
        self.zones_of_interest = {}
        self.num_cols = int(self.width/self.CELL_SIZE)
        self.num_rows = int(self.height/self.CELL_SIZE)
        self.grid = np.zeros((self.num_rows, self.num_cols))

    def addZoneOfInterest(self, zone:ZoneOfInterest) -> None:
        self.zones_of_interest[zone.id] = zone

    def getGridIndexes(self, x:float, y:float) -> Tuple[float, float]:
        '''Returns the indexes of the cell corresponding to position (x,y).'''
        row_index = math.ceil(y/self.CELL_SIZE)
        col_index = math.ceil(x/self.CELL_SIZE)

        if (row_index>= self.num_rows):
            row_index = self.num_rows-1

        if (row_index<0):
            row_index = 0

        if (col_index>= self.num_cols):
            col_index = self.num_cols -1

        if (col_index<0):
            col_index = 0
            
        return (row_index, col_index) 

    def getCellsIndexes(self, row_center:int, col_center:int, num_rows:int, num_cols:int) -> Tuple[int, int, int, int]:
        '''Returns the indexes of rows and cols around one given cell.'''
        row_min = max(0,row_center-math.ceil(num_rows/2))
        row_max = min(self.num_rows, row_center+math.ceil(num_rows/2)+1)
        col_min = max(0,col_center-math.ceil(num_cols/2))
        col_max = min(self.num_cols, col_center+math.ceil(num_cols/2)+1)
        return (row_min, row_max, col_min, col_max)

    def getCostCellGroup(self, n:int, m:int , num_rows:int, num_cols:int)->float:
        '''Returns the cost around some cell of the grid'''
        row_min, row_max, col_min, col_max = self.getCellsIndexes(n,m,num_rows, num_cols)
        sub_mat = self.grid[row_min:row_max, col_min:col_max]
        return np.sum(sub_mat)

    def getCellGroup(self, x:float, y:float, width:float, height:float) -> Tuple[int, int, int, int]:
        row_index,col_index = self.getGridIndexes(x, y)
        num_rows = math.ceil(height/self.CELL_SIZE)
        num_cols = math.ceil(width/self.CELL_SIZE)
        return (row_index, col_index, num_rows, num_cols)

    def getCostAroundPosition(self, x:float, y:float, width:float, height:float) -> float:
        '''Gets the current cost around some position (x,y)'''
        row_index, col_index, num_rows, num_cols = self.getCellGroup(x,y,width=width, height=height)
        print(f'Cost Around: {row_index}, {col_index}, {num_rows}, {num_cols}')
        return self.getCostCellGroup(row_index, col_index, num_rows=num_rows, num_cols=num_cols)

    def getCostOfZone(self, zone_id:string) -> float :
        '''Return the cost of a zone of interest'''
        if zone_id in self.zones_of_interest:
            zone = self.zones_of_interest[zone_id]
            return self.getCostAroundPosition(zone.x, zone.y, zone.width, zone.height)
        else:
            return -1

    def getCost(self) -> float:
        return np.sum(self.grid)

    def reduceCostFun(self, val:float, cost:float) -> float:
        reduced = val -cost
        if reduced<0:
            reduced = 0
        return reduced

    def reduceCost(self, cost: float) -> None:
        '''Applies the reduction function to the whole grid'''
        f = lambda x: self.reduceCostFun(x, cost)
        vectorize_f = np.vectorize(f)
        self.grid = vectorize_f(self.grid)


    def addCostFun(self, val:float, expansion:float, max_cost:float) -> float:
        cost = val + expansion
        if (cost>max_cost):
            cost = max_cost
        return cost

    def newDetection(self, x:float, y:float, expansion_fun: Callable, min_expansion_threshold:float, max_cost:float) -> None:
        '''Handles a new target detection
        
        Cost is added for the cells around the new position, until the cost is to small or
        the expansions is affecting the whole grid.
        '''
        row_index,col_index = self.getGridIndexes(x,y)
        #print(f'New detection in {row_index},{col_index}')
        end = False
        num_cells = 0
        while not end:
            expansion = expansion_fun(num_cells)
            if (expansion>min_expansion_threshold):
                row_min, row_max, col_min, col_max = self.getCellsIndexes(row_index,col_index,num_cells, num_cells)
                f = lambda val: self.addCostFun(val, expansion, max_cost)
                vectorize_f = np.vectorize(f)
                selection = self.grid[row_min:row_max, col_min:col_max]
                if (np.size(selection)>0):
                    expanded_mat = vectorize_f(selection)
                    self.grid[row_min:row_max, col_min:col_max] = expanded_mat
                    if (row_min==0 and col_min==0 and row_max == self.num_rows and row_min == self.num_cols):
                        end = True # If the expansion affected already the whole matrix, we stop
                else:
                    print('No cells')
            else:
                end = True
            num_cells = num_cells +1

    def getGridPositions(self) -> List:
        points = []
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                points.append(TargetPoint(col*self.CELL_SIZE, row*self.CELL_SIZE, self.grid[row,col], 0, 0))
        return points


    def getGridPositionsOfZone(self, zone_id:string) -> List:
        points = []
        if zone_id in self.zones_of_interest:
            zone = self.zones_of_interest[zone_id]
            row_index, col_index, num_rows, num_cols = self.getCellGroup(zone.x, zone.y, zone.width, zone.height)
            start_col = int(col_index-num_cols/2)
            start_row = int(row_index - num_rows/2)
            for row in range(num_rows):
                for col in range(num_cols):
                    points.append(TargetPoint((start_col + col)*self.CELL_SIZE, (start_row + row)*self.CELL_SIZE, self.grid[start_row + row,start_col + col], 0, 0))
        return points


class DoorGrid():
    CELL_SIZE = 0.2

    def __init__(self, size: DoorGridSize) -> None:
        self.occupancy_grid = OccupancyGrid(size.width, size.height)

        self.low_zoi = ZoneOfInterest(size.width/2, size.low_zone_height/2, size.width, size.low_zone_height,'low')
        self.high_zoi = ZoneOfInterest(size.width/2, size.height - size.high_zone_height/2, size.width, size.high_zone_height,'high')

        self.occupancy_grid.addZoneOfInterest(self.low_zoi)
        self.occupancy_grid.addZoneOfInterest(self.high_zoi)


    def is_in_low_or_high_zone(self,x:float, y:float)-> Tuple[bool,bool]:
        '''Return (True, True) if is in low, (True, False) if is in high, (False, False) if is not in low or high'''
        if (self.low_zoi.contains(x,y)):
            return (True, True)
        elif (self.high_zoi.contains(x,y)):
            return (True, False)

        return (False, False)

    def start(self, low_is_enter:bool=True):
        self.low_is_enter = low_is_enter

    def getEnterZoneCost(self) -> float:
        if (self.low_is_enter):
            return self.occupancy_grid.getCostOfZone('low')
        else:
            return self.occupancy_grid.getCostOfZone('high')

    def getExitZoneCost(self) -> float:
        if (not self.low_is_enter):
            return self.occupancy_grid.getCostOfZone('low')
        else:
            return self.occupancy_grid.getCostOfZone('high')

    def getGridPositionsHighzone(self) -> List:
        return self.occupancy_grid.getGridPositionsOfZone('high')

    def getGridPositionsLowzone(self) -> List:
        return self.occupancy_grid.getGridPositionsOfZone('low')

class DoorHandlerOptions():
    def __init__(self, reduce_fun: Callable, expansion_fun: Callable, complete_threshold:float, min_expansion_threshold:float, nearby_threshold:float, nearby_distance:float, delete_threshold:float, max_cost:float) -> None:
        self.reduce_fun = reduce_fun
        self.expansion_fun = expansion_fun
        self.complete_threshold = complete_threshold
        self.min_expansion_threshold = min_expansion_threshold
        self.nearby_threshold = nearby_threshold
        self.nearby_distance = nearby_distance
        self.delete_threshold = delete_threshold
        self.max_cost = max_cost

class TargetPoint():
    def __init__(self, x: float, y:float, z:float, ts:float, id:int) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.ts = ts
        self.id = id

class DoorGridHandler():
    def __init__(self, options: DoorHandlerOptions, grid_size: DoorGridSize) -> None:
        self.door_grids = []
        self.options = options
        self.grid_size = grid_size
        self.positions_stack = []
        self.last_detection_time_ms = time.time_ns()/1000

    def addPosition(self, pos: TargetPoint)-> None:
        self.positions_stack.append(pos)
        
    def reduceCost(self)->None:
        current_time_ms = time.time_ns()/1000
        elapsed_in_seconds = float(current_time_ms - self.last_detection_time_ms)/1000000
        self.last_detection_time_ms = current_time_ms
        cost_to_reduce = self.options.reduce_fun(elapsed_in_seconds)
        for door_grid in self.door_grids:
            door_grid.occupancy_grid.reduceCost(cost_to_reduce)

    def newDetection(self, pos: TargetPoint):
        pos_accepted = False
        if (len(self.door_grids)>0):
            for door_grid in self.door_grids:
                if (door_grid.occupancy_grid.getCostAroundPosition(pos.x, pos.y, self.options.nearby_distance, self.options.nearby_distance)>=self.options.nearby_threshold):
                    # This grid accepts the pos
                    pos_accepted = True
                    door_grid.occupancy_grid.newDetection(pos.x,pos.y, expansion_fun=self.options.expansion_fun, min_expansion_threshold=self.options.min_expansion_threshold, max_cost=self.options.max_cost)
            
        if (not pos_accepted):
            # Pos is not accepted by any grid, we check if we must start a new one
            door_grid = DoorGrid(self.grid_size)
            is_in_zone, low_is_enter = door_grid.is_in_low_or_high_zone(pos.x,pos.y)
            if (is_in_zone):
                print(f'New grid start low_is_enter: {low_is_enter} In position {pos.x},{pos.y}')
                door_grid.start(low_is_enter)
                self.door_grids.append(door_grid)
                door_grid.occupancy_grid.newDetection(pos.x,pos.y, expansion_fun=self.options.expansion_fun, min_expansion_threshold=self.options.min_expansion_threshold, max_cost=self.options.max_cost)
            

    def deleteEmptyGrids(self) -> None:
        active_grids = []
        
        for door_grid in self.door_grids:
                #print(f'Cost grid {door_grid.getCost()}')
                if (door_grid.occupancy_grid.getCost()>= self.options.delete_threshold):
                    #print(f'Grid keeps active')
                    active_grids.append(door_grid)
        self.door_grids = active_grids

    def getTransitions(self) -> Tuple[int,int]:
        active_grids = []
        low_to_high = 0
        high_to_low = 0
        for door_grid in self.door_grids:
            #print(f'Enter zone cost {door_grid.getEnterZoneCost()} Exit zone cost {door_grid.getExitZoneCost()}')
            if (door_grid.getEnterZoneCost()>= self.options.complete_threshold and door_grid.getExitZoneCost()>= self.options.complete_threshold):
                #This grid is complete. We add a new transition to the counter and delete the grid.
                print('New transition')
                if (door_grid.low_is_enter):
                    low_to_high = low_to_high +1
                else:
                    high_to_low = high_to_low +1
            else:
                #Grid is not completed, it keeps active
                active_grids.append(door_grid)
        self.door_grids = active_grids
        return (low_to_high, high_to_low)

    def loop(self) -> Tuple[int, int]:
        
        #We reduce the cost of all grids
        self.reduceCost()

        #We process the stacked positions
        for index in range(len(self.positions_stack)):
            new_pos = self.positions_stack.pop(0)
            self.newDetection(new_pos)

        # We check if some target went from low to high or high to low in the last position
        low_to_high, high_to_low = self.getTransitions()

        # We delete the empty grids if there are some
        self.deleteEmptyGrids()

        print(f'Active grids {len(self.door_grids)}')

        return (low_to_high, high_to_low)


class TimeOccupancyHandlerOptions():
    def __init__(self, reduce_fun: Callable, expansion_fun: Callable, min_expansion_threshold:float, delete_threshold:float, max_cost:float, occupancy_cost_threshold:float, occupancy_time:float, deoccupancy_time:float, max_occupancy_time:float) -> None:
        self.reduce_fun = reduce_fun
        self.expansion_fun = expansion_fun
        self.min_expansion_threshold = min_expansion_threshold
        self.delete_threshold = delete_threshold
        self.max_cost = max_cost
        self.occupancy_cost_threshold = occupancy_cost_threshold
        self.occupancy_time = occupancy_time
        self.deoccupancy_time = deoccupancy_time
        self.max_occupancy_time = max_occupancy_time

class TimeOccupancyHandler:
    def __init__(self, options: TimeOccupancyHandlerOptions, grid_size: GridSize) -> None:
        self.options = options
        self.grid_size = grid_size
        self.occupancy_grids = {}
        self.occupancy_zones = {}
        self.positions_stack = {}
        self.last_detection_time_ms = time.time_ns()/1000
        self.occupancy_zones_state = {}

    def addOccupancyZone(self, zone:ZoneOfInterest) -> None:
        self.occupancy_zones[zone.id] = zone
        self.occupancy_zones_state[zone.id] = {}

    def reduceCost(self, elapsed_in_seconds:float)->None:
        cost_to_reduce = self.options.reduce_fun(elapsed_in_seconds)
        for oc_grid_id in self.occupancy_grids.keys():
            oc_grid = self.occupancy_grids[oc_grid_id]
            print(f'Reduce grid Target: {oc_grid_id}  Cost: {cost_to_reduce}')
            oc_grid.reduceCost(cost_to_reduce)

    def newDetection(self, pos: TargetPoint) -> None:
        if (not pos.id in self.occupancy_grids):
            #We create a new occupancy grid for the new target id
            new_oc_grid = OccupancyGrid(self.grid_size.width, self.grid_size.height)
            for zone in self.occupancy_zones.values():
                new_oc_grid.addZoneOfInterest(zone)
            self.occupancy_grids[pos.id] = new_oc_grid
        oc_grid = self.occupancy_grids[pos.id]
        oc_grid.newDetection(pos.x,pos.y, expansion_fun=self.options.expansion_fun, min_expansion_threshold=self.options.min_expansion_threshold, max_cost=self.options.max_cost)
 
    def updateZonesState(self, elapsed_in_seconds: float, detections_last_frame: Dict) -> None: 

        updated_zones_state = {}

        for zone_id in self.occupancy_zones_state.keys():
            if (zone_id in detections_last_frame):
                curret_zone_state = self.occupancy_zones_state[zone_id]
                updated_zone_state = {}
                current_targets = self.occupancy_zones_state[zone_id].keys()
                last_frame_targets = detections_last_frame[zone_id]

                #First, we check targets already in the zone
                for current_target_id in current_targets:
                    if (current_target_id in last_frame_targets):
                        # In this frame a target continues inside this zone
                        last_frame_targets.remove(current_target_id) #We remove it from the list so we dont use it twice
                        updated_zone_state[current_target_id] = min(curret_zone_state[current_target_id] + elapsed_in_seconds, self.options.max_occupancy_time)
                    else:
                        #A target was in this zone, no anymore
                        new_time = curret_zone_state[current_target_id] - elapsed_in_seconds
                        if (new_time>0):
                            #If time<0, we dont add it to the list
                             updated_zone_state[current_target_id] = new_time

                #Next, we check if there is some new target in the zone
                for new_target_id in last_frame_targets:
                    updated_zone_state[new_target_id] = elapsed_in_seconds
                
                updated_zones_state[zone_id] = updated_zone_state
            else:
                updated_zones_state[zone_id] = {}


        self.occupancy_zones_state = updated_zones_state

    
    def getZonesWithTargetsInside(self) -> Dict:
        zones_with_targets = {}
        for oc_grid_id in self.occupancy_grids.keys():
            oc_grid = self.occupancy_grids[oc_grid_id]
            for zone in oc_grid.zones_of_interest.values():
                zone_cost = oc_grid.getCostOfZone(zone.id)
                print(f'Zone {zone.id} Cost: {zone_cost} Target Grid: {oc_grid_id}')
                if zone_cost> self.options.occupancy_cost_threshold:
                    if not zone.id in zones_with_targets:
                        zones_with_targets[zone.id] = []
                    zones_with_targets[zone.id].append(oc_grid_id)
        return zones_with_targets

    def deleteEmptyGrids(self) -> None:
        active_grids = {}
        for oc_grid_id in self.occupancy_grids.keys():
                if (self.occupancy_grids[oc_grid_id].getCost()>= self.options.delete_threshold):
                    active_grids[oc_grid_id] = self.occupancy_grids[oc_grid_id]

        self.occupancy_grids = active_grids

    def addPosition(self, pos: TargetPoint)-> None:
        self.positions_stack[pos.id] = pos

    def loop(self) -> List:

        current_time_ms = time.time_ns()/1000
        elapsed_in_seconds = float(current_time_ms - self.last_detection_time_ms)/1000000
        self.last_detection_time_ms = current_time_ms

        self.reduceCost(elapsed_in_seconds)

        #We process the stacked positions
        for new_pos in self.positions_stack.values():
            self.newDetection(new_pos)

        #These are the zone with some target inside during this frame
        zones_with_targets_this_frame = self.getZonesWithTargetsInside()

        #We update the times that each target has been inside each zone
        self.updateZonesState(elapsed_in_seconds, zones_with_targets_this_frame)

        #These are the zones where a target has ben inside for enough time
        occupied_zones = []

        for zone_id in self.occupancy_zones_state.keys():
            zone_state = self.occupancy_zones_state[zone_id]
            for target_id in zone_state.keys():
                target_time_in_zone = zone_state[target_id]
                if (target_time_in_zone>=self.options.occupancy_time):
                    #Has been in the zone for a long time
                    occupied_zones.append((zone_id, target_id))

        # We delete the empty grids if there are some
        self.deleteEmptyGrids()

        print(f'Occupied Zones {occupied_zones}')

        return occupied_zones


class SitDownHandlerOptions():
    def __init__(self, change_threshold_seconds:float, change_threshold_z_variation:float) -> None:
        self.change_threshold_seconds = change_threshold_seconds
        self.change_threshold_z_variation = change_threshold_z_variation

class SitDownGrid():
    def __init__(self) -> None:
        self.current_max_z = 0
        self.is_sit = False
        self.time_to_change = 0

    def newDetection(self, z:float, change_threshold_z_variation:float, change_threshold_seconds:float, time_elapsed:float) -> None:
        if (z>self.current_max_z):
            self.current_max_z = z

        if (self.is_sit):
            if (z>=self.current_max_z-self.current_max_z*change_threshold_z_variation):
                #We were seated and the new Z is above the stand up threshold
                self.time_to_change = self.time_to_change + time_elapsed
            else:
                self.time_to_change=0
        else:
            if (z<=self.current_max_z-self.current_max_z*change_threshold_z_variation):
                #We stood and the new Z is below the sit down threshold
                self.time_to_change = self.time_to_change + time_elapsed
            else:
                self.time_to_change=0

        print(f'z: {z} max_z:{self.current_max_z} Time to change: {self.time_to_change}')

        if (self.time_to_change>=change_threshold_seconds):
            self.is_sit = not self.is_sit
            self.time_to_change = 0

class SitDownHandler():
    def __init__(self, options: SitDownHandlerOptions) -> None:
        self.options = options
        self.last_detection_time_ms = time.time_ns()/1000
        self.positions_stack = {}
        self.sit_down_grids = {}
    
    def addPosition(self, pos: TargetPoint)-> None:
        self.positions_stack[pos.id] = pos
    
    def newDetection(self, pos: TargetPoint, time_elapsed:float) -> None:
        if (not pos.id in self.sit_down_grids):
            #We create a new sit down grid for the new target id
            new_sd_grid = SitDownGrid()
            self.sit_down_grids[pos.id] = new_sd_grid
        sd_grid = self.sit_down_grids[pos.id]
        sd_grid.newDetection(pos.z,change_threshold_z_variation=self.options.change_threshold_z_variation, 
        change_threshold_seconds=self.options.change_threshold_seconds,time_elapsed=time_elapsed )
 
    def loop(self) -> Dict:

        current_time_ms = time.time_ns()/1000
        elapsed_in_seconds = float(current_time_ms - self.last_detection_time_ms)/1000000
        self.last_detection_time_ms = current_time_ms

        #We process the stacked positions
        for new_pos in self.positions_stack.values():
            self.newDetection(new_pos, elapsed_in_seconds)

        #Check state
        is_sit_state = {}
        for id in self.sit_down_grids.keys():
            sd_grid = self.sit_down_grids[id]
            is_sit_state[id] = sd_grid.is_sit


        print(f'Seat state {is_sit_state}')

        return is_sit_state