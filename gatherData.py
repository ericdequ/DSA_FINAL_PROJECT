import geopandas
import matplotlib
import osmnx
import osmnx as ox
import matplotlib.pyplot as plt
import datetime as dt
import geopandas as gpd
import shapely
import osmnx.osm_content_handler
import osmnx.geo_utils
import osmnx.utils
import osmnx.stats
import osmnx.core
import osmnx.downloader
import io
import fiona
fiona.supported_drivers

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    osmnx.config()
    # *graph using polygon*
    west = -82.3740413
    east = -82.3334415
    north = 29.6562674
    south = 29.6253962
    polygon = osmnx.bbox_to_poly(north, south, east, west) #constructs shape of uf campus
    graph2 = osmnx.graph_from_polygon(polygon,
                                      network_type="all",
                                      simplify =True,
                                      retain_all=True
                                      )

    graph3 = osmnx.get_undirected(graph2) # transforms to simply connected graphs
    osmnx.save_graph_shapefile(graph3, "nodes&edges") # saves nodes and edges to shapefile for ArcGis manipulation
