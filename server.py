import mesa
import mesa_geo as mg

from mesa.visualization.modules import CanvasGrid
from model import GeoModel
from model import Commuter
from model import Building
from model import TransportCell

model_params = {
    "num_buildings":mesa.visualization.Slider("number of buildings",5,2,50),
    "num_commuters":mesa.visualization.Slider("number of agents",5,1,100),
    "num_destinations":mesa.visualization.Slider("number of destinations",3,2,5),
    "resolution":mesa.visualization.Slider("resolution",400,100,1000,10),
    "trace_fade":mesa.visualization.Choice("fade traces", True,[False,True]),
    "trace_strength":mesa.visualization.Slider("trace length",100,10,400,10)
}

def model_draw(item):
    """
    Portrayal Method for canvas determines the colors of each model element
    """
    portrayal = dict()
    if isinstance(item, Commuter):
        #agents are red
        portrayal["color"] = "Red"
        portrayal["radius"] = .5
    if isinstance(item, Building):
        #buildings are blue
        portrayal["color"] = "Blue"
        portrayal["radius"] = 1.0
    if isinstance(item, TransportCell):
        #traces left by agents are yellow
        if item.trace_strength == 0:
            #cell color is gray
            return 40, 70, 156, 1
        else:
            #
            return (
                255,
                255,
                0,
                1,
            )

    return portrayal


map_element = mg.visualization.MapModule(model_draw)


server = mesa.visualization.ModularServer(
    GeoModel, [map_element], "Commuter_model", model_params
)

#launch the model in web
server.port = 8525

server.launch()
