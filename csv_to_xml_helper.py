"""
    Script usage:
    call python csv_to_xml_helper.py <input csv obstacle file> <input csv route file> <output csv route file>
    the output file will automatically placed under output_xmls file
"""

import numpy as np
import pandas as pd
from rospkg import RosPack
import sys

def all_csv_to_xml(csv_filename1,csv_filename2,xml_filename):
    dr = pd.read_csv(csv_filename2)    #route
    map_ = dr.iloc[len(dr)-1][dr.keys()[0]]
    vehicle_ = dr.iloc[len(dr)-1][dr.keys()[1]]
    mode_ = dr.iloc[len(dr)-1][dr.keys()[2]]
    if mode_ == "with_obstacle":
        do = pd.read_csv(csv_filename1)    #obstacle
    
    with open(xml_filename,'w') as xml_file:

        xml_file.write("<?xml version=\"1.0\"?>\n<scenarios>\n")
        line1 = "\t<scenario name=\"ControlAssessment\" type=\"ControlAssessment\" town=\"{}\">\n".format(str(map_))
        xml_file.write(line1)
        line2 = "\t\t<ego_vehicle"
        for key in dr.keys():
            if len(dr) > 1:
                number =dr.iloc[0][key]
                line2 += " {}=\"{}\"".format(key, str(number)) 
        line2 += " {}=\"{}\"/>\n".format("model",str(vehicle_)) 
        xml_file.write(line2)
        if mode_ == "with_obstacle":
            for i in range(len(do)):
                line = "/t/t<object"
                for key in do.keys():
                    number = do.iloc[i][key]
                    if key == "object_name":
                        line += " {}=\"{}\"".format("model", str(number))
                    else:
                        line += " {}=\"{}\"".format(key, str(number))
                line += "/>\n"
                xml_file.write(line)
        xml_file.write("\t\t<route>\n")
        for i in range(len(dr)-1):
            line = "\t\t\t<waypoint"
            for key in dr.keys():
                number = dr.iloc[i][key]
                line += " {}=\"{}\"".format(key, str(number))
            line += "/>\n"
            xml_file.write(line)
        xml_file.write("\t\t</route>\n")
        xml_file.write("\t</scenario>\n</scenarios>")




        
if __name__ == "__main__":
    assert(len(sys.argv) == 4)
    input_name_obstacle = sys.argv[1]
    input_name_routes = sys.argv[2]
    output_name = sys.argv[3]
    rp = RosPack()
    path = rp.get_path('carla_scenario_runner_ros')
    path += "/src/carla_scenario_runner_ros/srunner/configs/"
    output_name = path + output_name
    all_csv_to_xml(input_name_obstacle,input_name_routes,output_name)


