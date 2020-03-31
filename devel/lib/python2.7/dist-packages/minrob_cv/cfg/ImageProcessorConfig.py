## *********************************************************
##
## File autogenerated for the minrob_cv package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'HSV', 'lower': 'hsv', 'srcline': 124, 'name': 'hsv', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::HSV', 'field': 'DEFAULT::hsv', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 22, 'description': 'Maximum hue.', 'max': 179, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 'h_max', 'edit_method': '', 'default': 179, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 23, 'description': 'Minimum hue.', 'max': 179, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 'h_min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 24, 'description': 'Maximum saturation.', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 's_max', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 25, 'description': 'Minimum saturation.', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 's_min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 26, 'description': 'Maximum value.', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 'v_max', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 27, 'description': 'Minimum value.', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/cfg/ImageProcessor.cfg', 'name': 'v_min', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 1}], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

