import math
from sys import gettrace
import numpy as np
from typing import Callable, Tuple, List
import time


class DoorGridSize():
    def __init__(self, width_meters: float, height_meters: float, low_zone_height: float, high_zone_height: float) -> None:
        self.width = width_meters
        self.height = height_meters
        self.low_zone_height = low_zone_height
        self.high_zone_height = high_zone_height

class DoorGrid():
    CELL_SIZE = 0.2

    def __init__(self, size: DoorGridSize) -> None:
        self.width = size.width
        self.height = size.height
        self.low_zone_height = size.low_zone_height
        self.high_zone_height = size.high_zone_height

        self.num_cols = int(self.width/self.CELL_SIZE)
        self.num_rows = int(self.height/self.CELL_SIZE)

        #print(f'New grid {self.num_rows}, {self.num_cols}')

        self.num_rows_low_zone = int(self.low_zone_height/self.CELL_SIZE)
        self.num_rows_high_zone = int(self.high_zone_height/self.CELL_SIZE)
        self.grid = np.zeros((self.num_rows, self.num_cols))


    def is_in_low_or_high_zone(self,x:float, y:float)-> Tuple[bool,bool]:
        if (x>=0 and x<self.width):
            if (y>=0 and y<self.low_zone_height):
                return (True, True)
            elif (y<self.height and y>=self.height-self.high_zone_height):
                return (True, False)
        
        return (False, False)

    def start(self, low_is_enter:bool=True):
        self.low_is_enter = low_is_enter

    def getGridIndexes(self, x:float, y:float) -> Tuple[float, float]:
        row_index = int(y/self.CELL_SIZE)
        col_index = int(x/self.CELL_SIZE)

        if (row_index>= self.num_rows):
            row_index = self.num_rows-1

        if (col_index>= self.num_cols):
            col_index = self.num_cols -1
            
        return (row_index, col_index)

    def getCostAroundPosition(self, x:float, y:float, radius:float) -> float:
        row_index,col_index = self.getGridIndexes(x,y)
        num_cells = int(radius/self.CELL_SIZE)
        return self.getCostAroundCell(row_index, col_index, num_cells)

    def getCostAroundCell(self, n:int, m:int , num_cells:int)->float:
        row_min, row_max, col_min, col_max = self.getRingIndexes(n,m,num_cells)
        sub_mat = self.grid[row_min:row_max, col_min:col_max]
        return np.sum(sub_mat)
    
    def getCost(self) -> float:
        return np.sum(self.grid)

    def getEnterZoneCost(self) -> float:
        if (self.low_is_enter):
            sub_mat = self.grid[0:self.num_rows_low_zone, 0:self.num_cols]
        else:
            sub_mat = self.grid[(self.num_rows - self.num_rows_high_zone):self.num_rows, 0:self.num_cols]
        return np.sum(sub_mat)

    def getExitZoneCost(self) -> float:
        if (not self.low_is_enter):
            sub_mat = self.grid[0:self.num_rows_low_zone, 0:self.num_cols]
        else:
            sub_mat = self.grid[(self.num_rows-self.num_rows_high_zone):self.num_rows, 0:self.num_cols]
        return np.sum(sub_mat)

    def reduceCostFun(self, val:float, cost:float) -> float:
        reduced = val -cost
        if reduced<0:
            reduced = 0
        return reduced

    def reduceCost(self, cost: float) -> None:
        f = lambda x: self.reduceCostFun(x, cost)
        vectorize_f = np.vectorize(f)
        self.grid = vectorize_f(self.grid)


    def addCostFun(self, val:float, expansion:float, max_cost:float) -> float:
        #print(f'In addCos val={val}')
        cost = val + expansion
        if (cost>max_cost):
            cost = max_cost
        return cost

    def newDetection(self, x:float, y:float, expansion_fun: Callable, min_expansion_threshold:float, max_cost:float) -> None:
        row_index,col_index = self.getGridIndexes(x,y)
        #print(f'New detection in {row_index},{col_index}')
        end = False
        num_cells = 0
        while not end:
            expansion = expansion_fun(num_cells)
            #print(f'Num Cells: {num_cells}, expansion: {expansion}')
            if (expansion>min_expansion_threshold):
                row_min, row_max, col_min, col_max = self.getRingIndexes(row_index,col_index,num_cells)
                #print(f'Ring Indexes: {row_min}, {row_max}, {col_min}, {col_max}')
                f = lambda val: self.addCostFun(val, expansion, max_cost)
                vectorize_f = np.vectorize(f)
                expanded_mat = vectorize_f(self.grid[row_min:row_max, col_min:col_max])
                #print(f'Expanded mat: {expanded_mat}')
                self.grid[row_min:row_max, col_min:col_max] = expanded_mat
                if (row_min==0 and col_min==0 and row_max == self.num_rows and row_min == self.num_cols):
                    end = True # If the expansion affected already the whole matrix, we stop
            else:
                end = True
            num_cells = num_cells +1

    def getRingIndexes(self, row_center:int, col_center:int, num_cells:int) -> Tuple[int, int, int, int]:
        row_min = max(0,row_center-num_cells)
        row_max = min(self.num_rows, row_center+num_cells+1)
        col_min = max(0,col_center-num_cells)
        col_max = min(self.num_cols, col_center+num_cells+1)
        return (row_min, row_max, col_min, col_max)

    def getGridPositions(self) -> List:
        points = []
        for row in range(self.num_rows):
            #print(f'{self.grid[row,:]}')
            for col in range(self.num_cols):
                points.append(TargetPoint(col*self.CELL_SIZE, row*self.CELL_SIZE, self.grid[row,col], 0, 0))
        #print(f'\n')
        return points

    def getGridPositionsHighzone(self) -> List:
        points = []
        for row in range(self.num_rows-self.num_rows_high_zone, self.num_rows):
            #print(f'{self.grid[row,:]}')
            for col in range(self.num_cols):
                points.append(TargetPoint(col*self.CELL_SIZE, row*self.CELL_SIZE, self.grid[row,col], 0, 0))
        #print(f'\n')
        return points

    def getGridPositionsLowzone(self) -> List:
        points = []
        for row in range(0, self.num_rows_low_zone):
            #print(f'{self.grid[row,:]}')
            for col in range(self.num_cols):
                points.append(TargetPoint(col*self.CELL_SIZE, row*self.CELL_SIZE, self.grid[row,col], 0, 0))
        #print(f'\n')
        return points


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
            door_grid.reduceCost(cost_to_reduce)

    def newDetection(self, pos: TargetPoint):
        pos_accepted = False
        if (len(self.door_grids)>0):
            for door_grid in self.door_grids:
                if (door_grid.getCostAroundPosition(pos.x, pos.y, self.options.nearby_distance)>=self.options.nearby_threshold):
                    # This grid accepts the pos
                    pos_accepted = True
                    door_grid.newDetection(pos.x,pos.y, expansion_fun=self.options.expansion_fun, min_expansion_threshold=self.options.min_expansion_threshold, max_cost=self.options.max_cost)
            
        if (not pos_accepted):
            # Pos is not accepted by any grid, we check if we must start a new one
            door_grid = DoorGrid(self.grid_size)
            is_in_zone, low_is_enter = door_grid.is_in_low_or_high_zone(pos.x,pos.y)
            if (is_in_zone):
                print(f'New grid start low_is_enter: {low_is_enter} In position {pos.x},{pos.y}')
                door_grid.start(low_is_enter)
                self.door_grids.append(door_grid)
                door_grid.newDetection(pos.x,pos.y, expansion_fun=self.options.expansion_fun, min_expansion_threshold=self.options.min_expansion_threshold, max_cost=self.options.max_cost)
            

    def deleteEmptyGrids(self) -> None:
        active_grids = []
        
        for door_grid in self.door_grids:
                #print(f'Cost grid {door_grid.getCost()}')
                if (door_grid.getCost()>= self.options.delete_threshold):
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






