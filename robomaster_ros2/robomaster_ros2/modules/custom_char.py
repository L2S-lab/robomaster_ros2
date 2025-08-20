import yaml
from ament_index_python.packages import get_package_share_directory
from os.path import join
PKG_NAME = 'robomaster_ros2'
CUSTOM_CHAR = {}
with open(join(get_package_share_directory(PKG_NAME), 'config_ros', "custom_char.yaml" ),"r") as file:
    CUSTOM_CHAR = yaml.safe_load(file)
    file.close()

def valid_custom_char(char):
    if char not in CUSTOM_CHAR.keys():
        return False
    ret = CUSTOM_CHAR.get(char)
    if len(ret) != 64:
        return False
    if not set(ret).issubset(set('rpb0')):
        return False
    return True

def get_custom_char(char, color='r'):
    ret = CUSTOM_CHAR.get(char)
    s = ['r' in ret, 'p' in ret, 'b' in ret]
    if s.count(True) == 1:
        if color=='':
            return ret
        elif 'r' in ret:
          return ret.replace('r',color)
        elif 'p' in ret:
          return ret.replace('p',color)
        elif 'b' in ret:
          return ret.replace('b',color)
    else:
        return ret