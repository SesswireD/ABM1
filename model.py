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

def get_cell(point: Point, model):
    """When given a point the function returns the index of the grid cell that contains this point"""
    resolution_x, resolution_y = model.space.raster_layer.resolution
    lower_left_x, lower_left_y = model.space.raster_layer.total_bounds[0:2]

    agent_x = point.x
    agent_y = point.y

    pos_x = int(np.floor((agent_x - lower_left_x)/resolution_x))
    pos_y = int(np.floor((agent_y - lower_left_y)/resolution_y))

    pos = (pos_x,pos_y)

    return pos

class TransportCell(mg.Cell):
    agents_total: int
    agents_last_steps: list
    velocity: float

    def __init__(
        self,
        pos: mesa.space.Coordinate | None = None,
        indices: mesa.space.Coordinate | None = None,
    ):
        super().__init__(pos, indices)
        self.agents_total = None
        self.agents_last_steps = None
        self.velocity = None

    def cell_used(self)->None:
        """Performs the actions required if an agent passes the cell"""
        self.agents_total = self.agents_total +1

    def step(self):
        pass
    
class TransportMap(mg.GeoSpace):

    def __init__(self, crs):
        super().__init__(crs=crs)
        
    def set_raster_layer(self, raster_file, crs):
        raster_layer = mg.RasterLayer.from_file(
            raster_file, cell_cls=TransportCell, attr_name="velocity"
            )
        raster_layer.crs = crs

        raster_layer.apply_raster(
            data=np.zeros(shape=(1, raster_layer.height, raster_layer.width)),
            attr_name="agents_total"
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
        print('building')



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
    destinations: list
    

    def __init__(self, unique_id, model, geometry, crs, speed = 0.1) -> None:
            super().__init__(unique_id, model, geometry, crs)
            self.speed = speed

    def move(self, new_location)->None:
        self.geometry = new_location

    def move_to_destination(self):
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
            # self.goal = self.home

        # set new goal?
        # else:
            # self.goal = self.home
    
    def move_to_destination_random(self):
        self.goal = self.destinations
        distance = self.geometry.distance(self.goal)

        # move into direction of goal with random deviation
        if distance > self.speed:
            dx = self.goal.x - self.geometry.x
            dy = self.goal.y - self.geometry.y

            # calculate direction
            direction = np.arctan2(dx,dy)
            direction = np.degrees(direction)

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
            # self.goal = self.home

    def leave_trail(self, old_position, new_position):
        
        line = LineString([old_position, new_position])
        line = gpd.GeoSeries(line)
        l = {'uniqueID': 0, 'geometry': line}
        line = gpd.GeoDataFrame(l, crs=crs)

        ac = mg.AgentCreator(agent_class=Trail, model= self.model)
        agent = ac.from_GeoDataFrame(line, unique_id="uniqueID")
        self.model.space.add_agents(agent)

    def leave_trace(self):
        """Function to leave a trace on the grid"""
        row, col = get_cell(self.geometry, self.model)
        cell = self.model.space.raster_layer[row][col]
        cell.cell_used()

    def step(self):
        old_position = (self.geometry.x, self.geometry.y)
        self.leave_trace()
        self.move_to_destination_random()
        new_position = (self.geometry.x, self.geometry.y)
        #self.leave_trail(old_position, new_position)


#the actual model defines the space and initializes agents
class GeoModel(mesa.Model):

    def __init__(self, num_buildings=20, num_commuters=15):
        self.schedule = mesa.time.RandomActivation(self)
        self.space = TransportMap(crs=crs)
        self.space.set_raster_layer("100x100_EPSG4326.tif", crs=crs)
        self.x_dim = self.space.raster_layer.width
        self.y_dim = self.space.raster_layer.height
        self.building = num_buildings
        self.commuters = num_commuters

        #Initialize an empty geodataframe for the buildings
        d = {'uniqueID': [], 'geometry': []}
        buildings = gpd.GeoDataFrame(d, crs=crs)

        #Create random points for buildings
        for i in range(num_buildings):
            bounds = self.space.raster_layer.total_bounds
            x = np.random.uniform(bounds[0], bounds[2])
            y = np.random.uniform(bounds[1], bounds[3])
            point = Point([x,y])
            point = gpd.GeoSeries(point)
            p = {'uniqueID': i, 'geometry': point}
            point = gpd.GeoDataFrame(p, crs=crs)
            buildings = pd.concat([buildings, point], ignore_index=True)

        #Place building object (GeoAgent) at the points in the GeoDataframe
        ac2 = mg.AgentCreator(agent_class=Building, model=self)
        agents2 = ac2.from_GeoDataFrame(buildings, unique_id="uniqueID")
        self.space.add_agents(agents2)

        # create commuters and assign them to buildings
        d = {'uniqueID': [], 'geometry': [], 'home': [], 'destination': []}
        commuters = gpd.GeoDataFrame(d, crs=crs)

        for i in range(num_commuters):
            places = buildings.sample(n = 2)
            home = places["geometry"].iloc[0]
            home_location = places["geometry"].iloc[0]
            destination = places["geometry"].iloc[1]

            c = {'uniqueID': i, 'geometry': home_location, 'home': home, 'destinations': [destination]}
            commuter = gpd.GeoDataFrame(c, crs=crs)
            commuters = pd.concat([commuters, commuter], ignore_index=True)

        ac1 = mg.AgentCreator(agent_class=Commuter, model=self)
        agents1 = ac1.from_GeoDataFrame(commuters, unique_id="uniqueID")

        self.space.add_agents(agents1)

        #add agents to scheduler
        self.schedule_Commuter = RandomActivation(self)
        for agent in agents1:
            self.schedule_Commuter.add(agent)

        
    def step(self) -> None:
        self.schedule_Commuter.step()
        #self.schedule_Trail.step()

    def run_model(self, step_count=10):
        for i in range(step_count):
            self.step()