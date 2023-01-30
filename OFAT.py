from mesa.batchrunner import BatchRunner
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
from model import GeoModel

############################################################################ OFAT analysis

# We define our variables and bounds
problem = {
    'num_vars': 3,#5,
    'names': ['num_buildings', 'num_destinations', 'trace_strength'],#, 'vision_angle', 'vision_samples'],
    'bounds': [[50,100,11], [2,10,9], [10,100,10]],#, [7.5,90,12], [3,15,7]]
    # 'bounds': [[50,100,2], [2,10,2], [10,100,2]],#, [7.5,90,2], [3,15,2]]
}

# Set the repetitions, the amount of steps, and the amount of distinct values per variable
replicates = 2
max_steps = 3

# Set the outputs
model_reporters = {'Avg_raster_value': lambda m: m.avg_raster_value}

data = pd.DataFrame()

for i, var in enumerate(problem['names']):
    # Get the bounds for this variable and get <distinct_samples> samples within this space (uniform)
    samples = np.linspace(*problem['bounds'][i], dtype=int)
    
    # # Keep in mind that wolf_gain_from_food should be integers. You will have to change
    # # your code to acommodate for this or sample in such a way that you only get integers.
    if var == 'vision_angle':
        samples = np.linspace(*problem['bounds'][i])
    
    batch = BatchRunner(GeoModel, 
                        max_steps=max_steps,
                        iterations=replicates,
                        variable_parameters={var: samples},
                        model_reporters=model_reporters,
                        display_progress=True)
    
    batch.run_all()
    
    data = pd.concat([data, batch.get_model_vars_dataframe()])

data.to_csv('OFAT_data.csv')

################################################################################### Plotting

def plot_param_var_conf(ax, df, var, param, i):
    """
    Helper function for plot_all_vars. Plots the individual parameter vs
    variables passed.

    Args:
        ax: the axis to plot to
        df: dataframe that holds the data to be plotted
        var: variables to be taken from the dataframe
        param: which output variable to plot
    """
    x = df.groupby(var).mean().reset_index()[var]
    y = df.groupby(var).mean()[param]

    replicates = df.groupby(var)[param].count()
    err = (1.96 * df.groupby(var)[param].std()) / np.sqrt(replicates)

    ax.plot(x, y, c='k')
    ax.fill_between(x, y - err, y + err)

    ax.set_xlabel(var)
    ax.set_ylabel(param)

def plot_all_vars(df, param):
    """
    Plots the parameters passed vs each of the output variables.

    Args:
        df: dataframe that holds all data
        param: the parameter to be plotted
    """

    f, axs = plt.subplots(problem['num_vars'], figsize=(7, 10))
    
    for i, var in enumerate(problem['names']):
        df_temp = df[~df[var].isna()]
        plot_param_var_conf(axs[i], df_temp, var, param, i)


plot_all_vars(data, 'Avg_raster_value')
plt.savefig('OFAT_plot.png', bbox_inches='tight')