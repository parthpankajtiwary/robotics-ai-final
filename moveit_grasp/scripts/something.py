# hardcoded =
# {'Evergreen' :
#     x: 0.215
#     y: 0.05
#     z: 0.095
# 'TomatoSoup' :
#     x: 0.1
#     y: 0.03
#     z: 0.13
# 'BaseTech' :
#     x: 0.16
#     y: 0.045
#     z: 0.17
# 'Eraserbox' :
#     x: 0.155
#     y: 0.05
#     z: 0.055
# 'UsbHub' :
#     x: 0.13
#     y: 0.07
#     z: 0.075}
#
# final_images_basetech
# final_images_usbhub
# final_images_eraserbox
# final_images_evergreen
# final_images_tomatosoup

hardcoded = [
    {'name' : 'BaseTech',
        'x': 0.16,
        'y': 0.045,
        'z': 0.17},
    {'name' : 'UsbHub',
        'x': 0.13,
        'y': 0.07,
        'z': 0.075},
    {'name' : 'Eraserbox',
        'x': 0.155,
        'y': 0.05,
        'z': 0.055},
    {'name' : 'Evergreen',
        'x': 0.215,
        'y': 0.05,
        'z': 0.095},
    {'name' : 'TomatoSoup',
        'x': 0.1,
        'y': 0.03,
        'z': 0.13}
]

import json
with open('objects.json', 'w') as file:
    json.dump(hardcoded, file)
