import mesa
import mesa_geo as mg
# import xyzservices.providers as xyz

from mesa.visualization.modules import CanvasGrid
from model import GeoModel
from model import Commuter
from model import Building
from model import Trail
from model import TransportCell
from model import TransportMap

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

def model_draw(item):
    """
    Portrayal Method for canvas
    """
    portrayal = dict()
    if isinstance(item, Commuter):
        portrayal["color"] = "Red"
        # portrayal["radius"] = "9"
    if isinstance(item, Trail):
        portrayal["color"] = "Pink"
    if isinstance(item, Building):
        portrayal["color"] = "Blue"
    if isinstance(item, TransportCell):
        if item.agents_total == 0:
            return 1, 1, 1, 0
        else:
            # return a blue color gradient based on the normalized water level
            # from the lowest water level colored as RGBA: (74, 141, 255, 1)
            # to the highest water level colored as RGBA: (0, 0, 255, 1)
            return (
                255,
                255,
                0,
                1,
            )

    return portrayal


map_element = mg.visualization.MapModule(model_draw)


server = mesa.visualization.ModularServer(
    GeoModel, [map_element], "Commuter_model", {}
)

#launch the model in web
# server.port = 8521

server.launch()
