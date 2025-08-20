import os 
import socket
import re
from subprocess import check_output

def wifi_off():
    os.system("nmcli radio wifi off")

def wifi_on():
    os.system("nmcli radio wifi on")

def connect(ssid,password=""):
    if password=="" or password==None:
        os.system(f"nmcli device wifi connect {ssid}")
    else:
        os.system(f"nmcli device wifi connect {ssid} password '{password}'")

def list():
    ls=os.popen("nmcli device wifi list").read()
    return ls

def get_psk(ssid):
    print(ssid)
    psk = os.popen(f"nmcli -t -f 802-11-wireless-security.psk c s {ssid} --show-secrets").read().split(':')[-1].split()[0]
    return psk

def share(opt=""):
    if opt == "qr":
        os.system("nmcli device wifi show-password")
        sh=0
    elif opt =="psk" or opt =="password" or  opt =="paskey":
        l = os.popen("nmcli device wifi show-password").read()
        sh=l.split("\n")[2].split(":")[1]
    else:
        sh=os.popen("nmcli device wifi show-password").read()
    return sh

def interface_list():
    i=os.listdir("/sys/class/net/")
    return i

def interface_status():
    in_s=os.popen("nmcli device status").read()
    in_s.replace(r"\n", " \n ")
    return in_s

def interface_config(i=""):
    if i!="":
        in_s=os.popen(f"iwconfig {i}").read()
    else:
        in_s=os.popen("ifconfig").read()
    return in_s

# TODO: Write a function to return SSID for the given IP address
# This is not working and not used in the code
def get_ssid(ip_address):
    try:
        hostname = socket.gethostbyaddr(ip_address)
        return hostname
    except socket.herror:
        return None

def get_interface_list(as_dict=False):
    device_lst = check_output(["nmcli", "device", "show"]).decode('utf-8').replace(" ", "").split('\n\n')
    devices = []
    for device in device_lst:
        devices.append(re.split(r'[\n:]', device))
    dict_devices = {"ethernets": [], "wifis": []}
    list_devices = []
    
    for i in range(len(devices)):
        try:
            _type = devices[i].index("GENERAL.TYPE")
            list_devices.append([devices[i][_type-1],
                                devices[i][devices[i].index("IP4.ADDRESS[1]")+1].split('/')[0],
                                devices[i][devices[i].index("GENERAL.CONNECTION")+1]])
            if devices[i][_type+1]=='ethernet':
                list_devices[-1].append("ethernet")
                dict_devices["ethernets"].append(list_devices[-1])
            elif devices[i][_type+1]=='wifi':
                list_devices[-1].append("wifi")
                dict_devices["wifis"].append(list_devices[-1])
            else:
                list_devices.pop()
        except:
            pass
    if as_dict:
        return dict_devices
    return list_devices

def interface_selector():
    interfaces = get_interface_list()
    menu_items = dict(enumerate(interfaces))
    while True:
        for item in menu_items:
            print(f"{item}: {menu_items[item]}")
        selection = input("Select an interface (number): ")
        if selection.isdigit() and int(selection) in menu_items:
            return menu_items[int(selection)]
        print("Invalid selection")
