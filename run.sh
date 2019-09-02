# ! /bin/bash
config_name="my_routes.csv"
scenario_name="ControlAssessment.xml"
echo "please design the scenario at first"
python 2d_routes_design.py $config_name
echo "load your configuration to scenario..."
python csv_to_xml_helper.py my_config.csv  $config_name $scenario_name
echo "load successfully"
