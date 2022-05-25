from grid_occupancy import DoorGridHandler, DoorGridSize, DoorHandlerOptions, TargetPoint
import threading



if __name__ == "__main__":

    grid_width = 2
    grid_height = 4
    grid_low_zone_height = 1
    grid_high_zone_height = 1

    grid_size = DoorGridSize(width_meters=grid_width, height_meters=grid_height, low_zone_height=grid_low_zone_height, high_zone_height=grid_high_zone_height)
    reduce_fun = lambda time_elapsed: time_elapsed * 10
    expansion_fun = lambda num_cells: 100/(num_cells +1)
    options = DoorHandlerOptions(reduce_fun=reduce_fun, expansion_fun=expansion_fun, complete_threshold=40, min_expansion_threshold=30, nearby_threshold=30, nearby_distance=2, delete_threshold=10)
    
    door_handler = DoorGridHandler(options=options, grid_size=grid_size)

    def main_loop():
        door_handler.loop()
        threading.Timer(1.0, main_loop).start()
