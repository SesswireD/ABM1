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
crs = "EPSG:3857"

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
    

    def __init__(self, unique_id, model, geometry, crs, speed = 100) -> None:
            super().__init__(unique_id, model, geometry, crs)
            self.speed = speed

    def move(self):
        self.goal = self.destinations
        distance = self.geometry.distance(self.goal)

        # move into direction of goal
        if distance > self.speed:
            dx = self.goal.x - self.geometry.x
            dy = self.goal.y - self.geometry.y
            new_x = self.geometry.x + dx / distance * self.speed
            new_y = self.geometry.y + dy / distance * self.speed
            self.geometry = Point(new_x, new_y)

        # move to goal when close enough
        elif distance > 0:
            self.geometry = self.goal
            # self.goal = self.home

        # set new goal?
        # else:
            # self.goal = self.home

    def leave_trail(self, old_position, new_position):
        
        line = LineString([old_position, new_position])
        line = gpd.GeoSeries(line)
        l = {'uniqueID': 0, 'geometry': line}
        line = gpd.GeoDataFrame(l, crs=crs)

        ac = mg.AgentCreator(agent_class=Trail, model= self.model)
        agent = ac.from_GeoDataFrame(line, unique_id="uniqueID")
        self.model.space.add_agents(agent)

    def step(self):
        old_position = (self.geometry.x, self.geometry.y)
        self.move()
        new_position = (self.geometry.x, self.geometry.y)
        self.leave_trail(old_position, new_position)


#the actual model defines the space and initializes agents
class GeoModel(mesa.Model):

    def __init__(self, x_dim=1,y_dim=1, num_buildings=10, num_commuters=2):

        self.space = mg.GeoSpace()
        self.schedule = mesa.time.RandomActivation(self)
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.building = num_buildings
        self.commuters = num_commuters

        #Initialize an empty geodataframe for the buildings
        d = {'uniqueID': [], 'geometry': []}
        buildings = gpd.GeoDataFrame(d, crs=crs)

        #Create random points
        for i in range(num_buildings):
            points = np.random.random((2))
            points = points*10000
            point = Point(points)
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

        
    def step(self):
        self.schedule_Commuter.step()

    # def run_model(self, step_count=200):
    #     for i in range(step_count):
    #         self.step()









