import os
os.environ['USE_PYGEOS'] = '0'
import pandas as pd
import geopandas as gpd
import mesa
import mesa_geo as mg
from shapely.geometry import Point, LineString
import pyproj
import numpy as np
from shapely.plotting import plot_line
from mesa.time import RandomActivation

#reference system for Mesa-Geo
crs = "EPSG:4326"

# define sample space for directions
deviations = np.linspace(-60, 60, 25)

def get_cell(point: Point, model):
    """When given a point the function returns the index of the grid cell that contains this point"""
    resolution_x, resolution_y = model.space.raster_layer.resolution
    lower_left_x, lower_left_y = model.space.raster_layer.total_bounds[0:2]

    agent_x = point.x
    agent_y = point.y

    pos_x = int(np.floor((agent_x - lower_left_x)/resolution_x))-1
    pos_y = int(np.floor((agent_y - lower_left_y)/resolution_y))-1

    pos = (pos_x,pos_y)

    return pos

def get_cells_in_direction(origin: Point, direction, distance, interval, model):
    """gets the cells under a line in a given direction by sampling"""
    n_points = int(np.ceil(distance/interval)) # number of points given by distance and sampling interval
    interval = distance/n_points # update interval length
    points = [] # should the original point be included?
    cells = []
    for i in range(1,n_points+1):
        x = origin.x + np.sin(np.radians(direction)) * i * interval
        y = origin.y + np.cos(np.radians(direction)) * i * interval
        point = Point([x,y])
        points.append(point)
        cell = get_cell(point, model)
        cells.append(cell)

    return points, cells

def get_direction(start: Point, goal: Point):
    """Takes two points and calculates the direction in degrees from start to goal"""

    #calculate change in x and y
    dx = goal.x - start.x
    dy = goal.y - start.y
    
    # calculate direction
    direction = np.arctan2(dx,dy)
    direction = np.degrees(direction)

    return direction

def get_utility(deviation: int, distance: float, cells: list, model):

    # get velocity values from cells
    velocity = 0
    for cell in cells:
        row, column = cell
        velocity += model.space.raster_layer[row][column].velocity

    # calculate some utility based on deviation, distance and velocity values
    utility = velocity * 200 + (60 - abs(deviation)) * (10 - distance) 

    return utility


class TransportCell(mg.Cell):
    agents_total: int
    agents_last_steps: list
    velocity: int

    def __init__(
        self,
        pos: mesa.space.Coordinate or None = None,
        indices: mesa.space.Coordinate or None = None,
    ):
        super().__init__(pos, indices)
        self.agents_total = 0
        self.agents_last_steps = None
        # self.velocity = 0

    def passed(self)->None:
        """Performs the actions required if an agent passes the cell"""
        self.agents_total += 1
        self.velocity += 10

    def step(self):
        if self.velocity > 0:
            self.velocity -= 1
            
    
class TransportMap(mg.GeoSpace):

    def __init__(self, crs):
        super().__init__(crs=crs)
    
    def set_raster_from_file(self, raster_file, crs):
        """load a raster file with velocity data and add the other empty attribute grids"""
        raster_layer = mg.RasterLayer.from_file(
            raster_file, cell_cls=TransportCell, attr_name="velocity"
            )
        raster_layer.crs = crs

        # generate the initial 0 grid for the number of agents that used a cell
        raster_layer.apply_raster(
            data=np.zeros(shape=(1, raster_layer.height, raster_layer.width)),
            attr_name="agents_total"
        )

        super().add_layer(raster_layer)

    def set_raster_layer(self, width, height, crs, bounds):
        """attempt to create raster without existing file"""

        #changing
        raster_layer = mg.RasterLayer(width,height,crs, total_bounds=[0.0,0.0,bounds,bounds],cell_cls=TransportCell)

        # generate the initial 0 grid for the number of agents that used a cell
        raster_layer.apply_raster(
            data=np.zeros(shape=(1, raster_layer.height, raster_layer.width)),
            attr_name="velocity"
        )

        super().add_layer(raster_layer)
    
    @property
    def raster_layer(self):
        return self.layers[0]

#buildings are defined as GeoAgent
class Building(mg.GeoAgent):
    unique_id: int
    model: mesa.Model
    geometry: Point
    crs: pyproj.CRS

    def __init__(self, unique_id, model, geometry, crs) -> None:
            super().__init__(unique_id, model, geometry, crs)

    def step(self):
        #print('building')
        pass



#Trail agent defines a road on the plane
class Trail(mg.GeoAgent):
    unique_id: int
    model: mesa.Model
    geometry: LineString
    crs: pyproj.CRS
    life: int
    cost: float

    def __init__(self, unique_id, model, geometry, crs, life = 10, cost = 10) -> None:
            super().__init__(unique_id, model, geometry, crs)

    def not_used(self):
        """If trail is not used it loses life"""
        self.life = self.life - 1
    
    def used(self):
        """If trail is used, its life is restored and its cost lowered"""
        self.life = 10

        if self.cost > 1:
            self.cost = self.cost - 1

    def step(self):
        # check whether it was used
        # run required method
        pass

#Commuter agents defines a moving person on the plane
class Commuter(mg.GeoAgent):
    unique_id: int
    model: mesa.Model
    geometry: Point
    crs: pyproj.CRS
    speed: float
    home: Building
    goal: Building
    destinations: list
    destination_count: int

    def __init__(self, unique_id, model, geometry, crs, speed = 0.3, destination_count = 0) -> None:
            super().__init__(unique_id, model, geometry, crs)
            self.speed = speed
            self.destination_count = destination_count

    def move(self, new_location)->None:
        self.geometry = new_location

    def move_to_destination(self):
        """Moves the agent towards a point in a straight line"""
        self.goal = self.destinations
        distance = self.geometry.distance(self.goal)

        # move into direction of goal
        if distance > self.speed:
            dx = self.goal.x - self.geometry.x
            dy = self.goal.y - self.geometry.y
            new_x = self.geometry.x + dx / distance * self.speed
            new_y = self.geometry.y + dy / distance * self.speed
            self.move(Point(new_x, new_y))

        # move to goal when close enough
        elif distance > 0:
            self.move(self.goal)

        #check if arrived and chance destination
        self.check_destination

    def move_to_destination_random(self):
        """Moves the agent towards a point with some random offset"""

        #get the distance to the current objective
        distance = self.geometry.distance(self.goal)

        # move into direction of goal with random deviation
        if distance > self.speed:

            direction = get_direction(self.geometry, self.goal)

            # draw deviation from normal distribution and add to the direction
            deviation = np.random.normal(0,20)
            direction = direction + deviation
            direction = np.deg2rad(direction)

            # calculate the next point
            new_x = self.geometry.x + np.sin(direction) * self.speed
            new_y = self.geometry.y + np.cos(direction) * self.speed
            self.move(Point(new_x, new_y))

        # move to goal when close enough
        elif distance > 0:
            self.move(self.goal)
            
        #check if arrived and chance destination
        self.check_destination
        

    def move_to_destination_preference(self):
        """moves the agent towards a point with"""
        
        distance = self.geometry.distance(self.goal)

        # move into direction of goal with random deviation
        if distance > self.speed:

            direction = get_direction(self.geometry, self.goal)

            # calculate utility for each direction +- 5 degrees
            utilities = np.zeros(len(deviations))
            for i, deviation in enumerate(deviations):
                
                test_direction = direction + deviation

                # get grid cells for this direction
                _, cells = get_cells_in_direction(self.geometry, test_direction, self.speed, self.speed/10, self.model)
                cells = set(cells)

                # calculate utility
                utilities[i] = get_utility(deviation, distance, cells, self.model)

            # draw random choice using utilities as weights
            probabilities = utilities / sum(utilities)
            deviation = np.random.choice(deviations, p=probabilities)

            direction = direction + deviation
            direction = np.deg2rad(direction)

            # calculate the next point
            new_x = self.geometry.x + np.sin(direction) * self.speed
            new_y = self.geometry.y + np.cos(direction) * self.speed
            self.move(Point(new_x, new_y))

        # move directly to goal when close enough
        elif distance > 0:
            self.move(self.goal)
            # self.goal = self.home

        #check if agent has arrived at destination and assign new destination
        self.check_destination()

                
    def check_destination(self):
        """Checks wether agent has arrived at current destination. If true, assign new destination"""

         #check if arrived at destination
        if self.geometry == self.goal:
            #check if all destinations visited
            if self.destination_count >= len(self.destinations) -1:
                
                #reset destination count
                self.destination_count = 0

                #return to home
                self.goal = self.home
            elif self.goal == self.home:
                self.goal = self.destinations[self.destination_count]
            else:
                #increment destination count
                self.destination_count += 1
                self.goal = self.destinations[self.destination_count] 


    def leave_trail(self, old_position, new_position):
        """Function to leave a trail on the grid. Trail defined as GeoAgent"""
        
        line = LineString([old_position, new_position])
        line = gpd.GeoSeries(line)
        l = {'uniqueID': 0, 'geometry': line}
        line = gpd.GeoDataFrame(l, crs=crs)

        ac = mg.AgentCreator(agent_class=Trail, model= self.model)
        agent = ac.from_GeoDataFrame(line, unique_id="uniqueID")
        self.model.space.add_agents(agent)

    def leave_trace(self, old, new) -> None:
        """Function to leave a trace on the grid. Trace defined as pass count in a raster"""
        direction = get_direction(old, new)
        distance = np.sqrt((new.x-old.x)**2 + (new.y-old.y)**2)

        _, cells_passed = get_cells_in_direction(old, direction, distance, self.speed/50, self.model)

        cells_passed = set(cells_passed)
        for cell in cells_passed:
            row, column = cell
            self.model.space.raster_layer[row][column].passed()

    def step(self):
        old_position = self.geometry
        self.move_to_destination_preference()
        new_position = self.geometry
        self.leave_trace(old_position,new_position)
        #self.leave_trail(old_position, new_position)


#the actual model defines the space and initializes agents
class GeoModel(mesa.Model):

    def __init__(self, num_buildings=10, num_commuters=5, num_destinations = 3, width=100, height=100, bounds=10.0):
        self.schedule = mesa.time.RandomActivation(self)
        self.space = TransportMap(crs=crs)
        # self.space.set_raster_from_image("1000x1000_EPSG4326.tif", crs=crs)
        self.space.set_raster_layer(width,width,crs, bounds)
        self.x_dim = self.space.raster_layer.width
        self.y_dim = self.space.raster_layer.width
        self.building = num_buildings
        self.commuters = num_commuters
        self.num_destinations = num_destinations

        #initialize random locations and place them on the map
        buildings = self.initialize_locations(num_buildings)

        #initialize agents and assign them destinations
        self.initialize_agents(num_commuters,num_destinations,buildings)


    def step(self) -> None:
        self.schedule_Commuter.step()
        #self.schedule_Trail.step()

    def run_model(self, step_count=10):
        for i in range(step_count):
            self.step()

    def initialize_locations(self,num_buildings):
        """Initializes a random number of locations on the raster.
        Locations are defined as Building GeoAgents

        returns:
        a GeoDataFrame with building locations
        """
        #let us know what the model is doing
        print("initializing locations")

        #Initialize an empty geodataframe for the buildings
        d = {'uniqueID': [], 'geometry': []}
        buildings = gpd.GeoDataFrame(d, crs=crs)

        #Create random points for buildings
        for i in range(num_buildings):

            #get the bounds from the model
            bounds = self.space.raster_layer.total_bounds

            #redefine bounds so buildings are created away from the borders
            x_min = bounds[0] + (0.05*bounds[2])
            x_max = bounds[2] * 0.95
            y_min = bounds[1] + (0.05*bounds[3])
            y_max = bounds[3] * 0.95

            #draw random coordinates
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)

            #add coordinates to the building dataframe
            point = Point([x,y])
            point = gpd.GeoSeries(point)
            p = {'uniqueID': i, 'geometry': point}
            point = gpd.GeoDataFrame(p, crs=crs)
            buildings = pd.concat([buildings, point], ignore_index=True)

        #create building agents from the locations in the dataframe
        ac2 = mg.AgentCreator(agent_class=Building, model=self)
        agents2 = ac2.from_GeoDataFrame(buildings, unique_id="uniqueID")

        #add the buildings to the space
        self.space.add_agents(agents2)

        return buildings

    def initialize_agents(self, num_commuters, num_destinations, buildings):

        # create empty geodf for commuters
        d = {'uniqueID': [], 'geometry': [], 'home': [], 'goal':[], 'destinations': []}
        commuters = gpd.GeoDataFrame(d, crs=crs)

        print("initializing agents")
        #create num_commuters commuters
        for i in range(num_commuters):

            #sample points to assign as destinations
            places = buildings.sample(n = num_destinations)

            #first sample point is home (and starting position of agent)
            home = places["geometry"].iloc[0]
            home_location = places["geometry"].iloc[0]

            #other sample points are travel destinations
            destination = places["geometry"].tolist()[1:]

            #first destination is the first goal
            goal = destination[0]

            #save info for the commuter agent in dictionary
            c = {'uniqueID': i, 'geometry': home_location, 'home': home,'goal': goal,'destinations':[destination]}

            #convert dictionary to geo dataframe
            commuter = gpd.GeoDataFrame(c, crs=crs)

            #add current commuter agent info to previous rows
            commuters = pd.concat([commuters, commuter], ignore_index=True)

        #instantiate commuter agents from geo dataframe
        ac1 = mg.AgentCreator(agent_class=Commuter, model=self)
        agents1 = ac1.from_GeoDataFrame(commuters, unique_id="uniqueID")

        #don't commuters need to be added to the space as well?
        self.space.add_agents(agents1)

        #add agents to scheduler
        self.schedule_Commuter = RandomActivation(self)
        for agent in agents1:
            self.schedule_Commuter.add(agent)

# model = GeoModel(num_buildings=2,num_commuters=1,num_destinations=2)
# model.run_model(step_count=3)