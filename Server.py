import mesa
import mesa_geo as mg
# import xyzservices.providers as xyz

from mesa.visualization.modules import CanvasGrid
from model import GeoModel
from model import Commuter
from model import Building
from model import Trail

def schelling_draw(agent):
    """
    Portrayal Method for canvas
    """
    portrayal = dict()
    if agent.atype is Commuter:
        portrayal["color"] = "Red"
    elif agent.atype == Building:
        portrayal["color"] = "Blue"
    else:
        portrayal["color"] = "Pink"
    return portrayal

def commuter_draw(agent):
    """
    Portrayal Method for canvas
    """
    portrayal = dict()
    if isinstance(agent, Commuter):
        portrayal["color"] = "Red"
        # portrayal["radius"] = "9"
    if isinstance(agent, Trail):
        portrayal["color"] = "Pink"
    if isinstance(agent, Building):
        portrayal["color"] = "Blue"
    return portrayal


map_element = mg.visualization.MapModule(commuter_draw)


server = mesa.visualization.ModularServer(
    GeoModel, [map_element], "Commuter_model", {}
)

#launch the model in web
# server.port = 8521

server.launch()